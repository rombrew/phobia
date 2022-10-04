#ifndef _H_SERIAL_
#define _H_SERIAL_

#define SERIAL_DEVICE_MAX		100
#define SERIAL_MEMORY_SIZE		8192

#define SERIAL_DEFAULT			"8E1"

enum {
	SERIAL_OK		= 0,
	SERIAL_ASYNC_WAIT	= -1,
	SERIAL_ERROR_UNKNOWN	= -2,
};

struct serial_list {

	const char		*name[SERIAL_DEVICE_MAX];
	int			dnum;

	char			mb[SERIAL_MEMORY_SIZE];
	char			*mbflow;
};

struct serial_fd;

void serial_enumerate(struct serial_list *ls);
struct serial_fd *serial_open(const char *devname, int baudrate, const char *mode);
void serial_close(struct serial_fd *fd);

int serial_async_fputs(struct serial_fd *fd, const char *s);
int serial_async_fgets(struct serial_fd *fd, char *s, int n);

#endif /* _H_SERIAL_ */

