#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <SDL2/SDL.h>

#ifdef _WINDOWS
#include <windows.h>
#else /* _WINDOWS */
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif /* _WINDOWS */

#include "serial.h"

struct serial_fd {

#ifdef _WINDOWS
	HANDLE		hFile;
#else
	int		port;
#endif /* _WINDOWS */

	struct async_priv	*rxq;
	struct async_priv	*txq;

	SDL_Thread	*thread_rxq;
	SDL_Thread	*thread_txq;
};

struct async_priv {

	int		length;
	int		cached;

	char		*stream;
	SDL_atomic_t	rp;
	SDL_atomic_t	wp;

	char		*ophunk;
	int		chunk;

	SDL_atomic_t	terminate;
};

static struct async_priv *
async_open(int length)
{
	struct async_priv	*ap;

	ap = calloc(1, sizeof(struct async_priv));

	ap->length = length;
	ap->stream = (char *) malloc(ap->length);

	ap->chunk = 80;
	ap->ophunk = (char *) malloc(ap->chunk);

	return ap;
}

static void
async_close(struct async_priv *ap)
{
	free(ap->stream);
	free(ap->ophunk);
	free(ap);
}

static int
async_getc(struct async_priv *ap)
{
	int		rp, wp, cq;

	rp = SDL_AtomicGet(&ap->rp);
	wp = SDL_AtomicGet(&ap->wp);

	if (rp != wp) {

		cq = (int) ap->stream[rp];
		rp = (rp < ap->length - 1) ? rp + 1 : 0;

		SDL_AtomicSet(&ap->rp, rp);

		return cq;
	}
	else {
		return SERIAL_ASYNC_WAIT;
	}
}

static int
async_putc(struct async_priv *ap, int cq)
{
	int		rp, wp, wpi;

	rp = SDL_AtomicGet(&ap->rp);
	wp = SDL_AtomicGet(&ap->wp);

	wpi = (wp < ap->length - 1) ? wp + 1 : 0;

	if (wpi != rp) {

		ap->stream[wp] = (char) cq;

		SDL_AtomicSet(&ap->wp, wpi);

		return SERIAL_OK;
	}
	else {
		return SERIAL_ASYNC_WAIT;
	}
}

static int
async_fgets(struct async_priv *ap, char *sbuf, int n)
{
	int		rp, wp, cq, eol, nq;

	rp = SDL_AtomicGet(&ap->rp);
	wp = SDL_AtomicGet(&ap->wp);

	if (wp != ap->cached) {

		eol = 0;
		nq = 0;

		do {
			if (rp == wp)
				break;

			cq = (int) ap->stream[rp];

			if (cq == '\r' || cq == '\n') {

				eol = (nq > 0) ? 1 : 0;
			}
			else if (eol == 1) {

				break;
			}
			else if (nq < n - 1) {

				*sbuf++ = (char) cq;
				nq++;
			}

			rp = (rp < ap->length - 1) ? rp + 1 : 0;
		}
		while (1);

		if (eol != 0) {

			ap->cached = rp;

			*sbuf = 0;

			SDL_AtomicSet(&ap->rp, rp);

			return SERIAL_OK;
		}
		else {
			ap->cached = rp;

			return SERIAL_ASYNC_WAIT;
		}
	}
	else {
		return SERIAL_ASYNC_WAIT;
	}
}

static int
async_space(struct async_priv *ap)
{
	int		rp, wp, nr;

	rp = SDL_AtomicGet(&ap->rp);
	wp = SDL_AtomicGet(&ap->wp);

	nr = wp - rp;
	nr += (nr < 0) ? ap->length : 0;

	return ap->length - nr;
}

#ifdef _WINDOWS
void serial_enumerate(struct serial_list *ls)
{
	char		*lpPath, *lpName, *lpCOM;
	DWORD		comTake, uMax = 131072U;

	memset(ls, 0, sizeof(struct serial_list));

	ls->mbflow = ls->mb;

	lpPath = malloc(uMax);

	do {
		if (QueryDosDeviceA(NULL, lpPath, uMax) != 0)
			break;

		if (uMax < 8388608U) {

			uMax *= 2U;
			lpPath = realloc(lpPath, uMax);
		}
		else {
			/* We are unable to get DOS device names.
			 * */
			return ;
		}
	}
	while(1);

	lpName = lpPath;

	while (*lpName != 0) {

		lpCOM = strstr(lpName, "COM");

		if (lpCOM != NULL) {

			comTake = 0;

			if (lpCOM[3] >= '0' && lpCOM[3] <= '9') {

				if (lpCOM[4] == 0) {

					comTake = 1;
				}
				else if (lpCOM[4] >= '0' && lpCOM[4] <= '9') {

					if (lpCOM[5] == 0) {

						comTake = 1;
					}
				}
			}

			if (comTake != 0) {

				sprintf(ls->mbflow, "%s", lpName);

				ls->name[ls->dnum] = ls->mbflow;

				ls->mbflow += strlen(ls->mbflow) + 1;
				ls->dnum++;

				if (ls->dnum >= SERIAL_DEVICE_MAX)
					break;
			}
		}

		lpName += strlen(lpName) + 1;
	}

	free(lpPath);

	if (ls->dnum == 0) {

		ls->mbflow[0] = 0;
		ls->name[0] = ls->mbflow;

		ls->dnum = 1;
	}
}

static struct serial_fd *
serial_port_open(const char *devname, int baudrate, const char *mode)
{
	struct serial_fd	*fd;

	char			lpName[80];

	HANDLE			hFile;
	COMMTIMEOUTS		CommTimeouts;
	DCB			CommDCB;

	sprintf(lpName, "//./%.75s", devname);

	hFile = CreateFileA(lpName, GENERIC_READ | GENERIC_WRITE,
			0, NULL, OPEN_EXISTING, 0, NULL);

	if (hFile == INVALID_HANDLE_VALUE)
		return NULL;

	SetupComm(hFile, 1024, 1024);

	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutConstant = 100;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 0;

	SetCommTimeouts(hFile, &CommTimeouts);
	SetCommMask(hFile, EV_ERR);

	memset(&CommDCB, 0, sizeof(CommDCB));
	CommDCB.DCBlength = sizeof(CommDCB);

	GetCommState(hFile, &CommDCB);

	switch (mode[0]) {

		case '5':
			CommDCB.ByteSize = 5;
			break;

		case '6':
			CommDCB.ByteSize = 6;
			break;

		case '7':
			CommDCB.ByteSize = 7;
			break;

		default:
		case '8':
			CommDCB.ByteSize = 8;
			break;
	}

	switch (mode[1]) {

		default:
		case 'N':
			CommDCB.Parity = NOPARITY;
			break;

		case 'O':
			CommDCB.Parity = ODDPARITY;
			break;

		case 'E':
			CommDCB.Parity = EVENPARITY;
			break;

	}

	switch (mode[2]) {

		default:
		case '1':
			CommDCB.StopBits = ONESTOPBIT;
			break;

		case '2':
			CommDCB.StopBits = TWOSTOPBITS;
			break;
	}

	switch (baudrate) {

		case 1200: CommDCB.BaudRate = CBR_1200; break;
		case 2400: CommDCB.BaudRate = CBR_2400; break;
		case 4800: CommDCB.BaudRate = CBR_4800; break;
		case 9600: CommDCB.BaudRate = CBR_9600; break;
		case 19200: CommDCB.BaudRate = CBR_19200; break;
		case 38400: CommDCB.BaudRate = CBR_38400; break;
		case 57600: CommDCB.BaudRate = CBR_57600; break;
		case 115200: CommDCB.BaudRate = CBR_115200; break;
		case 128000: CommDCB.BaudRate = CBR_128000; break;
		case 256000: CommDCB.BaudRate = CBR_256000; break;
		default: break;
	}

	CommDCB.fOutxCtsFlow = FALSE;
	CommDCB.fOutxDsrFlow = FALSE;
	CommDCB.fDtrControl = DTR_CONTROL_DISABLE;
	CommDCB.fDsrSensitivity = FALSE;
	CommDCB.fTXContinueOnXoff = FALSE;
	CommDCB.fOutX = FALSE;
	CommDCB.fInX = FALSE;
	CommDCB.fErrorChar = FALSE;
	CommDCB.fNull = FALSE;
	CommDCB.fRtsControl = RTS_CONTROL_DISABLE;
	CommDCB.fAbortOnError = FALSE;

	SetCommState(hFile, &CommDCB);

	fd = calloc(1, sizeof(struct serial_fd));

	fd->hFile = hFile;

	return fd;
}

static int
serial_port_read(struct serial_fd *fd, char *s, int n)
{
	BOOL		bRet;
	DWORD		nBytes = 0;

	bRet = ReadFile(fd->hFile, (LPVOID) s, (DWORD) n, &nBytes, NULL);

	if (bRet == FALSE || nBytes < 1) {

		return SERIAL_ERROR_UNKNOWN;
	}

	return (int) nBytes;
}

static int
serial_port_write(struct serial_fd *fd, const char *s, int n)
{
	BOOL		bRet;
	DWORD		nBytes, nTotal = 0;

	while (nTotal < n) {

		nBytes = 0;

		bRet = WriteFile(fd->hFile, (LPCVOID) (s + nTotal),
				(DWORD) (n - nTotal), &nBytes, NULL);

		if (bRet == FALSE || nBytes < 1) {

			return SERIAL_ERROR_UNKNOWN;
		}

		nTotal += nBytes;
	}

	return (int) nTotal;
}

#else /* _WINDOWS */
static void
serial_port_pattern(struct serial_list *ls, const char *pat)
{
	DIR			*dir;
	struct dirent		*en;

	int			len;

	dir = opendir("/dev");
	len = strlen(pat);

	if (dir != NULL) {

		while ((en = readdir(dir)) != NULL) {

			if (en->d_type == DT_CHR || en->d_type == DT_UNKNOWN) {

				if (memcmp(en->d_name, pat, len) == 0) {

					sprintf(ls->mbflow, "/dev/%s", en->d_name);

					ls->name[ls->dnum] = ls->mbflow;

					ls->mbflow += strlen(ls->mbflow) + 1;
					ls->dnum++;

					if (ls->dnum >= SERIAL_DEVICE_MAX)
						break;
				}
			}
		}

		closedir(dir);
	}
}

void serial_enumerate(struct serial_list *ls)
{
	memset(ls, 0, sizeof(struct serial_list));

	ls->mbflow = ls->mb;

	serial_port_pattern(ls, "ttyACM");
	serial_port_pattern(ls, "ttyUSB");
	serial_port_pattern(ls, "rfcomm");
	serial_port_pattern(ls, "ttyS");

#ifdef __APPLE__
	serial_port_pattern(ls, "tty.BT");
	serial_port_pattern(ls, "tty.Bluetooth");
	serial_port_pattern(ls, "tty.usbmodem");
#endif /* __APPLE__ */

	if (ls->dnum == 0) {

		ls->mbflow[0] = 0;
		ls->name[0] = ls->mbflow;

		ls->dnum = 1;
	}
}

static struct serial_fd	*
serial_port_open(const char *devname, int baudrate, const char *mode)
{
	struct serial_fd	*fd;

	struct termios		tio;
	int			port;

	port = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);

	if (port < 0)
		return NULL;

	memset(&tio, 0, sizeof(tio));
	tcgetattr(port, &tio);

	tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR
			| IGNCR | ICRNL | IXON | IXOFF | IXANY | IGNPAR);
	tio.c_oflag &= ~(OPOST | ONLCR | OCRNL);
	tio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_lflag &= ~(ECHO | ECHOK | ECHONL | ECHOCTL | ECHOKE
			| ICANON | ISIG | IEXTEN);

	switch (mode[0]) {

		case '5':
			tio.c_cflag |= CS5;
			break;

		case '6':
			tio.c_cflag |= CS6;
			break;

		case '7':
			tio.c_cflag |= CS7;
			break;

		default:
		case '8':
			tio.c_cflag |= CS8;
			break;
	}

	switch (mode[1]) {

		default:
		case 'N':
			break;

		case 'O':
			tio.c_cflag |= PARENB | PARODD;
			break;

		case 'E':
			tio.c_cflag |= PARENB;
			break;
	}

	switch (mode[2]) {

		default:
		case '1':
			break;

		case '2':
			tio.c_cflag |= CSTOPB;
			break;
	}

	switch (baudrate) {

		case 1200: baudrate = B1200; break;
		case 2400: baudrate = B2400; break;
		case 4800: baudrate = B4800; break;
		case 9600: baudrate = B9600; break;
		case 19200: baudrate = B19200; break;
		case 38400: baudrate = B38400; break;
		case 57600: baudrate = B57600; break;
		case 115200: baudrate = B115200; break;
		case 230400: baudrate = B230400; break;
		default: break;
	}

	cfsetospeed(&tio, baudrate);
	cfsetispeed(&tio, baudrate);

	tio.c_cc[VTIME] = 1;
	tio.c_cc[VMIN]  = 0;

	if (tcsetattr(port, TCSANOW, &tio) != 0) {

		close(port);
		return NULL;
	}

	fd = calloc(1, sizeof(struct serial_fd));

	fd->port = port;

	return fd;
}

static int
serial_port_read(struct serial_fd *fd, char *s, int n)
{
	n = read(fd->port, s, n);

	if (n < 1) {

		return SERIAL_ERROR_UNKNOWN;
	}

	return n;
}

static int
serial_port_write(struct serial_fd *fd, const char *s, int n)
{
	int		rc, total = 0;

	while (total < n) {

		rc = write(fd->port, s + total, n - total);

		if (rc < 1) {

			return SERIAL_ERROR_UNKNOWN;
		}

		total += rc;
	}

	return total;
}
#endif /* _WINDOWS */

static int
async_thread_rx(struct serial_fd *fd)
{
	struct async_priv	*ap = fd->rxq;
	int			cq, rc, n, i;

	do {
		n = serial_port_read(fd, ap->ophunk, ap->chunk);

		for (i = 0; i < n; ++i) {

			cq = (int) ap->ophunk[i];

			do {
				rc = async_putc(ap, cq);

				if (rc == SERIAL_OK)
					break;

				SDL_Delay(10);
			}
			while (1);
		}

		SDL_Delay(1);

		if (SDL_AtomicGet(&ap->terminate) != 0)
			break;
	}
	while (1);

	async_close(ap);

	return 0;
}

static int
async_thread_tx(struct serial_fd *fd)
{
	struct async_priv	*ap = fd->txq;
	int			cq, rc, n, terminate = 0;

	do {
		n = 0;

		if (SDL_AtomicGet(&ap->terminate) != 0) {

			terminate = 1;
		}

		do {
			cq = async_getc(ap);

			if (cq == SERIAL_ASYNC_WAIT)
				break;

			ap->ophunk[n++] = (char) cq;

			if (n >= ap->chunk)
				break;
		}
		while (1);

		if (n > 0) {

			rc = serial_port_write(fd, ap->ophunk, n);

			if (rc != n) {

				/* TODO */
			}
		}
		else {
			SDL_Delay(10);

			if (terminate != 0)
				break;
		}
	}
	while (1);

	async_close(ap);

	return 0;
}

struct serial_fd *serial_open(const char *devname, int baudrate, const char *mode)
{
	struct serial_fd	*fd;

	fd = serial_port_open(devname, baudrate, mode);

	if (fd != NULL) {

		fd->rxq = async_open(4000);
		fd->txq = async_open(200);

		fd->thread_rxq = SDL_CreateThread((int (*) (void *)) &async_thread_rx,
				"async_thread_rx", fd);

		fd->thread_txq = SDL_CreateThread((int (*) (void *)) &async_thread_tx,
				"async_thread_tx", fd);
	}

	return fd;
}

static int
serial_thread_garbage(struct serial_fd *fd)
{
	SDL_WaitThread(fd->thread_txq, NULL);

#ifdef _WINDOWS
	CloseHandle(fd->hFile);
#else
	close(fd->port);
#endif /* _WINDOWS */

	SDL_WaitThread(fd->thread_rxq, NULL);

	free(fd);

	return 0;
}

void serial_close(struct serial_fd *fd)
{
	SDL_Thread	*thread;

	SDL_AtomicSet(&fd->rxq->terminate, 1);
	SDL_AtomicSet(&fd->txq->terminate, 1);

	thread = SDL_CreateThread((int (*) (void *)) &serial_thread_garbage,
			"serial_thread_garbage", fd);

	SDL_DetachThread(thread);
}

int serial_fputs(struct serial_fd *fd, const char *s)
{
	int		av, rc;

	av = async_space(fd->txq);

	if (strlen(s) < av) {

		while (*s != 0) {

			rc = async_putc(fd->txq, *s++);

			if (rc != SERIAL_OK) {

				return SERIAL_ERROR_UNKNOWN;
			}
		}

		return SERIAL_OK;
	}
	else {
		return SERIAL_ASYNC_WAIT;
	}
}

int serial_fgets(struct serial_fd *fd, char *s, int n)
{
	return async_fgets(fd->rxq, s, n);
}

