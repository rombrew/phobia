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

struct async_priv {

	struct serial_fd	*fd;
	SDL_Thread		*thread;

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
async_open(struct serial_fd *fd, int length)
{
	struct async_priv	*ap;

	ap = calloc(1, sizeof(struct async_priv));

	ap->fd = fd;
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
async_space_available(struct async_priv *ap)
{
	int		rp, wp, nr;

	rp = SDL_AtomicGet(&ap->rp);
	wp = SDL_AtomicGet(&ap->wp);

	nr = wp - rp;
	nr += (nr < 0) ? ap->length : 0;

	return ap->length - nr;
}

static int serial_raw_read(struct serial_fd *fd, char *s, int n);
static int serial_raw_write(struct serial_fd *fd, const char *s, int n);

static int
async_thread_RX(struct async_priv *ap)
{
	int		cq, rc, n, i;

	do {
		n = serial_raw_read(ap->fd, ap->ophunk, ap->chunk);

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
	}
	while (SDL_AtomicGet(&ap->terminate) == 0);

	async_close(ap);

	return 0;
}

static int
async_thread_TX(struct async_priv *ap)
{
	int		cq, rc, n;

	do {
		n = 0;

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

			rc = serial_raw_write(ap->fd, ap->ophunk, n);

			if (rc != n) {

				/* TODO */
			}
		}
		else {
			SDL_Delay(10);
		}
	}
	while (SDL_AtomicGet(&ap->terminate) == 0);

	async_close(ap);

	return 0;
}

#ifdef _WINDOWS

struct serial_fd {

	HANDLE			hFile;

	struct async_priv	*rxq;
	struct async_priv	*txq;
};

void serial_enumerate(struct serial_list *ls)
{
	char		*lpPath, *lpName, *lpCOM;
	int		comTake, uMax = 131072;

	memset(ls, 0, sizeof(struct serial_list));

	ls->mbflow = ls->mb;

	lpPath = malloc(uMax);

	do {
		if (QueryDosDeviceA(NULL, lpPath, uMax) != 0)
			break;

		if (uMax < 8388608) {

			uMax *= 2;
			lpPath = realloc(lpPath, uMax);
		}
		else {
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
}

struct serial_fd *serial_open(const char *devname, int baudrate, const char *mode)
{
	struct serial_fd	*fd;

	char			lpName[80];

	HANDLE			hFile;
	COMMTIMEOUTS		CommTimeouts;
	DCB			CommDCB;

	sprintf(lpName, "//./%.77s", devname);

	hFile = CreateFileA(lpName, GENERIC_READ | GENERIC_WRITE,
			0, NULL, OPEN_EXISTING, 0, NULL);

	if (hFile == INVALID_HANDLE_VALUE) {

		return NULL;
	}

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

		case 1200: baudrate = CBR_1200; break;
		case 2400: baudrate = CBR_2400; break;
		case 4800: baudrate = CBR_4800; break;
		case 9600: baudrate = CBR_9600; break;
		case 19200: baudrate = CBR_19200; break;
		case 38400: baudrate = CBR_38400; break;
		case 57600: baudrate = CBR_57600; break;
		case 115200: baudrate = CBR_115200; break;
		case 128000: baudrate = CBR_128000; break;
		case 256000: baudrate = CBR_256000; break;
		default: break;
	}

	CommDCB.BaudRate = (DWORD) baudrate;
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

	fd->rxq = async_open(fd, 4000);
	fd->txq = async_open(fd, 200);

	fd->rxq->thread = SDL_CreateThread((int (*) (void *)) &async_thread_RX,
			"async_thread_RX", fd->rxq);

	fd->txq->thread = SDL_CreateThread((int (*) (void *)) &async_thread_TX,
			"async_thread_TX", fd->txq);

	return fd;
}

void serial_close(struct serial_fd *fd)
{
	SDL_AtomicSet(&fd->rxq->terminate, 1);
	SDL_AtomicSet(&fd->txq->terminate, 1);

	SDL_DetachThread(fd->rxq->thread);
	SDL_DetachThread(fd->txq->thread);

	CloseHandle(fd->hFile);

	free(fd);
}

int serial_raw_read(struct serial_fd *fd, char *s, int n)
{
	BOOL		bRet;
	DWORD		nBytes = 0;

	bRet = ReadFile(fd->hFile, s, n, &nBytes, NULL);

	if (bRet == FALSE || nBytes < 1) {

		return SERIAL_ERROR_UNKNOWN;
	}

	return (int) nBytes;
}

int serial_raw_write(struct serial_fd *fd, const char *s, int n)
{
	BOOL		bRet;
	DWORD		nBytes, nTotal = 0;

	while (nTotal < n) {

		nBytes = 0;

		bRet = WriteFile(fd->hFile, s + nTotal, n - nTotal, &nBytes, NULL);

		if (bRet == FALSE || nBytes < 1) {

			return SERIAL_ERROR_UNKNOWN;
		}

		nTotal += nBytes;
	}

	return (int) nTotal;
}

#else /* _WINDOWS */

struct serial_fd {

	int			port;

	struct async_priv	*rxq;
	struct async_priv	*txq;
};

static void
serial_scan_pattern(struct serial_list *ls, const char *pat)
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

	serial_scan_pattern(ls, "ttyACM");
	serial_scan_pattern(ls, "ttyUSB");
	serial_scan_pattern(ls, "rfcomm");
	serial_scan_pattern(ls, "ttyS");
}

struct serial_fd *serial_open(const char *devname, int baudrate, const char *mode)
{
	struct serial_fd	*fd;

	struct termios		tio;
	int			port;

	port = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);

	if (port < 0) {

		return NULL;
	}

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

	fd->rxq = async_open(fd, 4000);
	fd->txq = async_open(fd, 200);

	fd->rxq->thread = SDL_CreateThread((int (*) (void *)) &async_thread_RX,
			"async_thread_RX", fd->rxq);

	fd->txq->thread = SDL_CreateThread((int (*) (void *)) &async_thread_TX,
			"async_thread_TX", fd->txq);

	return fd;
}

void serial_close(struct serial_fd *fd)
{
	SDL_AtomicSet(&fd->rxq->terminate, 1);
	SDL_AtomicSet(&fd->txq->terminate, 1);

	SDL_DetachThread(fd->rxq->thread);
	SDL_DetachThread(fd->txq->thread);

	close(fd->port);

	free(fd);
}

static int
serial_raw_read(struct serial_fd *fd, char *s, int n)
{
	n = read(fd->port, s, n);

	if (n < 1) {

		return SERIAL_ERROR_UNKNOWN;
	}

	return n;
}

static int
serial_raw_write(struct serial_fd *fd, const char *s, int n)
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

int serial_async_fputs(struct serial_fd *fd, const char *s)
{
	int		av, rc;

	av = async_space_available(fd->txq);

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

int serial_async_fgets(struct serial_fd *fd, char *s, int n)
{
	return async_fgets(fd->rxq, s, n);
}

