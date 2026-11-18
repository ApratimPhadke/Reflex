/* rxfs - QNX resource manager that puts Reflex into the pathname space.
 *
 *     cat /dev/reflex                  live status, same text as `rxctl status`
 *     echo "fault flip" > /dev/reflex  inject a fault
 *     echo "guard off"  > /dev/reflex  bypass the guardian (demo only)
 *     echo "sp 50"      > /dev/reflex  move the setpoint to +50 mm
 *
 * It is a separate, ordinary-priority process on purpose: in a microkernel the
 * observability layer is just another server. If rxfs crashes or is killed, the
 * guardian does not notice.
 *
 * Needs the PROCMGR_AID_PATHSPACE ability, i.e. start it as root.
 *
 * QNX only. SPDX-License-Identifier: MIT */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/stat.h>
#include "rx_shm.h"

static rx_shm_t *shm;
static resmgr_connect_funcs_t connect_funcs;
static resmgr_io_funcs_t      io_funcs;
static iofunc_attr_t          attr;
static char                   text[4096];
static int                    text_len;

static int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
    int status = iofunc_read_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

    if (ocb->offset == 0) text_len = rx_status_text(shm, text, sizeof text);   /* snapshot per open/rewind */

#ifdef _IO_READ_GET_NBYTES
    size_t want = _IO_READ_GET_NBYTES(msg);
#else
    size_t want = msg->i.nbytes;
#endif
    size_t left = ocb->offset < text_len ? (size_t)(text_len - ocb->offset) : 0;
    size_t n = want < left ? want : left;

    _IO_SET_READ_NBYTES(ctp, n);
    if (n == 0) return _RESMGR_NPARTS(0);                                       /* EOF */
    SETIOV(ctp->iov, text + ocb->offset, n);
    ocb->offset += n;
    return _RESMGR_NPARTS(1);
}

static int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
    int status = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

#ifdef _IO_WRITE_GET_NBYTES
    size_t nbytes = _IO_WRITE_GET_NBYTES(msg);
#else
    size_t nbytes = msg->i.nbytes;
#endif
    char cmd[128];
    size_t n = nbytes < sizeof cmd - 1 ? nbytes : sizeof cmd - 1;
    if (resmgr_msgread(ctp, cmd, n, sizeof msg->i) == -1) return errno;
    cmd[n] = 0;
    cmd[strcspn(cmd, "\r\n")] = 0;

    _IO_SET_WRITE_NBYTES(ctp, nbytes);
    if (cmd[0] && rx_command(shm, cmd) != 0) return EINVAL;                     /* shell prints "Invalid argument" */
    return _RESMGR_NPARTS(0);
}

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "/dev/reflex";
    while (!(shm = rx_shm_open(0))) { fprintf(stderr, "rxfs: waiting for guardian...\n"); sleep(1); }

    dispatch_t *dpp = dispatch_create();
    if (!dpp) { perror("rxfs: dispatch_create"); return 1; }

    resmgr_attr_t rattr;
    memset(&rattr, 0, sizeof rattr);
    rattr.nparts_max = 1;
    rattr.msg_max_size = 2048;

    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
    io_funcs.read = io_read;
    io_funcs.write = io_write;
    iofunc_attr_init(&attr, S_IFNAM | 0666, NULL, NULL);

    if (resmgr_attach(dpp, &rattr, path, _FTYPE_ANY, 0, &connect_funcs, &io_funcs, &attr) == -1) {
        perror("rxfs: resmgr_attach (are you root?)");
        return 1;
    }
    fprintf(stderr, "rxfs: serving %s\n", path);

    dispatch_context_t *ctp = dispatch_context_alloc(dpp);
    for (;;) {
        ctp = dispatch_block(ctp);
        if (!ctp) { perror("rxfs: dispatch_block"); return 1; }
        dispatch_handler(ctp);
    }
}
