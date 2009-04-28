/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * file and vm operations
 */

#include <linux/fs_stack.h>
#include <linux/poll.h>
#include "aufs.h"

/* common function to regular file and dir */
int aufs_flush(struct file *file, fl_owner_t id)
{
	int err;
	aufs_bindex_t bindex, bend;
	struct dentry *dentry;
	struct file *h_file;

	dentry = file->f_dentry;
	si_noflush_read_lock(dentry->d_sb);
	fi_read_lock(file);
	di_read_lock_child(dentry, AuLock_IW);

	err = 0;
	bend = au_fbend(file);
	for (bindex = au_fbstart(file); !err && bindex <= bend; bindex++) {
		h_file = au_h_fptr(file, bindex);
		if (!h_file || !h_file->f_op || !h_file->f_op->flush)
			continue;

		err = h_file->f_op->flush(h_file, id);
		if (!err)
			vfsub_update_h_iattr(&h_file->f_path, /*did*/NULL);
		/*ignore*/
	}
	au_cpup_attr_timesizes(dentry->d_inode);

	di_read_unlock(dentry, AuLock_IW);
	fi_read_unlock(file);
	si_read_unlock(dentry->d_sb);
	return err;
}

/* ---------------------------------------------------------------------- */

static int do_open_nondir(struct file *file, int flags)
{
	int err;
	aufs_bindex_t bindex;
	struct file *h_file;
	struct dentry *dentry;

	err = 0;
	dentry = file->f_dentry;
	au_fi(file)->fi_h_vm_ops = NULL;
	bindex = au_dbstart(dentry);
	/* O_TRUNC is processed already */
	BUG_ON(au_test_ro(dentry->d_sb, bindex, dentry->d_inode)
	       && (flags & O_TRUNC));

	h_file = au_h_open(dentry, bindex, flags, file);
	if (IS_ERR(h_file))
		err = PTR_ERR(h_file);
	else {
		au_set_fbstart(file, bindex);
		au_set_fbend(file, bindex);
		au_set_h_fptr(file, bindex, h_file);
		au_update_figen(file);
		/* todo: necessary? */
		/* file->f_ra = h_file->f_ra; */
	}
	return err;
}

static int aufs_open_nondir(struct inode *inode __maybe_unused,
			    struct file *file)
{
	return au_do_open(file, do_open_nondir);
}

static int aufs_release_nondir(struct inode *inode __maybe_unused,
			       struct file *file)
{
	struct super_block *sb = file->f_dentry->d_sb;

	si_noflush_read_lock(sb);
	au_finfo_fin(file);
	si_read_unlock(sb);
	return 0;
}

/* ---------------------------------------------------------------------- */

static ssize_t aufs_read(struct file *file, char __user *buf, size_t count,
			 loff_t *ppos)
{
	ssize_t err;
	struct dentry *dentry;
	struct file *h_file;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	h_file = au_h_fptr(file, au_fbstart(file));
	err = vfsub_read_u(h_file, buf, count, ppos);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);
 out:
	si_read_unlock(sb);
	return err;
}

static ssize_t aufs_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	ssize_t err;
	aufs_bindex_t bstart;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct super_block *sb;
	struct file *h_file;
	char __user *buf = (char __user *)ubuf;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	inode = dentry->d_inode;
	mutex_lock(&inode->i_mutex);
	si_read_lock(sb, AuLock_FLUSH);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	bstart = au_fbstart(file);
	h_file = au_h_fptr(file, bstart);
	au_unpin(&pin);
	err = vfsub_write_u(h_file, buf, count, ppos);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}

static ssize_t aufs_splice_read(struct file *file, loff_t *ppos,
				struct pipe_inode_info *pipe, size_t len,
				unsigned int flags)
{
	ssize_t err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	err = -EINVAL;
	h_file = au_h_fptr(file, au_fbstart(file));
	if (au_test_loopback_kthread()) {
		file->f_mapping = h_file->f_mapping;
		smp_mb(); /* unnecessary? */
	}
	err = vfsub_splice_to(h_file, ppos, pipe, len, flags);
	/* todo: necessasry? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	return err;
}

static ssize_t
aufs_splice_write(struct pipe_inode_info *pipe, struct file *file, loff_t *ppos,
		  size_t len, unsigned int flags)
{
	ssize_t err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct super_block *sb;
	struct file *h_file;

	dentry = file->f_dentry;
	inode = dentry->d_inode;
	mutex_lock(&inode->i_mutex);
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	h_file = au_h_fptr(file, au_fbstart(file));
	au_unpin(&pin);
	err = vfsub_splice_from(pipe, h_file, ppos, len, flags);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}

/* ---------------------------------------------------------------------- */

static struct file *au_safe_file(struct vm_area_struct *vma)
{
	struct file *file;

	file = vma->vm_file;
	if (file->private_data && au_test_aufs(file->f_dentry->d_sb))
		return file;
	return NULL;
}

static void au_reset_file(struct vm_area_struct *vma, struct file *file)
{
	vma->vm_file = file;
	/* smp_mb(); */ /* flush vm_file */
}

static int aufs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	struct file *file, *h_file;
	struct au_finfo *finfo;

	/* todo: non-robr mode, user vm_file as it is? */
	wait_event(wq, (file = au_safe_file(vma)));

	/* do not revalidate, no si lock */
	finfo = au_fi(file);
	h_file = finfo->fi_hfile[0 + finfo->fi_bstart].hf_file;
	AuDebugOn(!h_file || !au_test_mmapped(file));

	fi_write_lock(file);
	vma->vm_file = h_file;
	err = finfo->fi_h_vm_ops->fault(vma, vmf);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	au_reset_file(vma, file);
	fi_write_unlock(file);
#if 0 /* def CONFIG_SMP */
	/* wake_up_nr(&wq, online_cpu - 1); */
	wake_up_all(&wq);
#else
	wake_up(&wq);
#endif

	return err;
}

static struct vm_operations_struct aufs_vm_ops = {
	.fault		= aufs_fault
};

/* ---------------------------------------------------------------------- */

static struct vm_operations_struct *au_vm_ops(struct file *h_file,
					      struct vm_area_struct *vma)
{
	struct vm_operations_struct *vm_ops;
	int err;

	vm_ops = ERR_PTR(-ENODEV);
	if (!h_file->f_op || !h_file->f_op->mmap)
		goto out;

	err = h_file->f_op->mmap(h_file, vma);
	vm_ops = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	vm_ops = vma->vm_ops;
	err = do_munmap(current->mm, vma->vm_start,
			vma->vm_end - vma->vm_start);
	if (unlikely(err)) {
		AuIOErr("failed internal unmapping %.*s, %d\n",
			AuDLNPair(h_file->f_dentry), err);
		vm_ops = ERR_PTR(-EIO);
	}

 out:
	return vm_ops;
}

static int aufs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err;
	unsigned char wlock, mmapped;
	struct dentry *dentry;
	struct super_block *sb;
	struct file *h_file;
	struct vm_operations_struct *vm_ops;

	dentry = file->f_dentry;
	mmapped = !!au_test_mmapped(file); /* can be harmless race condition */
	wlock = !!(file->f_mode & FMODE_WRITE) && (vma->vm_flags & VM_SHARED);
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, wlock | !mmapped);
	if (unlikely(err))
		goto out;

	if (wlock) {
		struct au_pin pin;

		err = au_ready_to_write(file, -1, &pin);
		di_downgrade_lock(dentry, AuLock_IR);
		if (unlikely(err))
			goto out_unlock;
		au_unpin(&pin);
	} else if (!mmapped)
		di_downgrade_lock(dentry, AuLock_IR);

	h_file = au_h_fptr(file, au_fbstart(file));
	if (au_test_fs_bad_mapping(h_file->f_dentry->d_sb)) {
		/*
		 * by this assignment, f_mapping will differs from aufs inode
		 * i_mapping.
		 * if someone else mixes the use of f_dentry->d_inode and
		 * f_mapping->host, then a problem may arise.
		 */
		file->f_mapping = h_file->f_mapping;
	}

	vm_ops = NULL;
	if (!mmapped) {
		vm_ops = au_vm_ops(h_file, vma);
		err = PTR_ERR(vm_ops);
		if (IS_ERR(vm_ops))
			goto out_unlock;
	}

	/*
	 * unnecessary to handle MAP_DENYWRITE and deny_write_access()?
	 * currently MAP_DENYWRITE from userspace is ignored, but elf loader
	 * sets it. when FMODE_EXEC is set (by open_exec() or sys_uselib()),
	 * both of the aufs file and the lower file is deny_write_access()-ed.
	 * finally I hope we can skip handlling MAP_DENYWRITE here.
	 */
	err = generic_file_mmap(file, vma);
	if (unlikely(err))
		goto out_unlock;
	vma->vm_ops = &aufs_vm_ops;
	/* test again */
	if (!au_test_mmapped(file))
		au_fi(file)->fi_h_vm_ops = vm_ops;

	vfsub_file_accessed(h_file);
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	if (!wlock && mmapped)
		fi_read_unlock(file);
	else
		fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	return err;
}

/* ---------------------------------------------------------------------- */

static unsigned int aufs_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;
	int err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	/* We should pretend an error happened. */
	mask = POLLERR /* | POLLIN | POLLOUT */;
	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	/* it is not an error if h_file has no operation */
	mask = DEFAULT_POLLMASK;
	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->poll)
		mask = h_file->f_op->poll(h_file, wait);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	AuTraceErr((int)mask);
	return mask;
}

static int aufs_fsync_nondir(struct file *file, struct dentry *dentry,
			     int datasync)
{
	int err;
	struct au_pin pin;
	struct inode *inode;
	struct file *h_file;
	struct super_block *sb;

	inode = dentry->d_inode;
	IMustLock(file->f_mapping->host);
	if (inode != file->f_mapping->host) {
		mutex_unlock(&file->f_mapping->host->i_mutex);
		mutex_lock(&inode->i_mutex);
	}
	IMustLock(inode);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);

	err = 0; /* -EBADF; */ /* posix? */
	if (unlikely(!(file->f_mode & FMODE_WRITE)))
		goto out;
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;
	au_unpin(&pin);

	err = -EINVAL;
	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->fsync) {
		struct dentry *h_d;
		struct mutex *h_mtx;

		/*
		 * no filemap_fdatawrite() since aufs file has no its own
		 * mapping, but dir.
		 */
		h_d = h_file->f_dentry;
		h_mtx = &h_d->d_inode->i_mutex;
		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		err = h_file->f_op->fsync(h_file, h_d, datasync);
		if (!err)
			vfsub_update_h_iattr(&h_file->f_path, /*did*/NULL);
		/*ignore*/
		au_cpup_attr_timesizes(inode);
		mutex_unlock(h_mtx);
	}

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	if (inode != file->f_mapping->host) {
		mutex_unlock(&inode->i_mutex);
		mutex_lock(&file->f_mapping->host->i_mutex);
	}
	return err;
}

static int aufs_fasync(int fd, struct file *file, int flag)
{
	int err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->fasync)
		err = h_file->f_op->fasync(fd, h_file, flag);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	return err;
}

/* ---------------------------------------------------------------------- */

const struct file_operations aufs_file_fop = {
	/*
	 * while generic_file_llseek/_unlocked() don't use BKL,
	 * don't use it since it operates file->f_mapping->host.
	 * in aufs, it may be a real file and may confuse users by UDBA.
	 */
	/* .llseek		= generic_file_llseek, */

	.read		= aufs_read,
	.write		= aufs_write,
	.poll		= aufs_poll,
	.mmap		= aufs_mmap,
	.open		= aufs_open_nondir,
	.flush		= aufs_flush,
	.release	= aufs_release_nondir,
	.fsync		= aufs_fsync_nondir,
	.fasync		= aufs_fasync,
	.splice_write	= aufs_splice_write,
	.splice_read	= aufs_splice_read
};
