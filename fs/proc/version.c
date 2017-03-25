#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>

static int version_proc_show(struct seq_file *m, void *v)
{
#define HW_VERSION
#ifdef HW_VERSION
	/*
	 * Workaround
	 * refer to kernel/Makefile: the version is 3.xx.xx, but HW wanted 3.xx
	 */
	int i=0, dot_cnt=0;
	char release_buf[__NEW_UTS_LEN + 1] = {'\0'};
	strcpy(release_buf, utsname()->release);
	// .xx -> \0\0\0
	while (release_buf[i]) {
		if ('.' == release_buf[i]) {
			dot_cnt++;
			if (2 == dot_cnt)
				release_buf[i] = '\0';
		}
		i++;
	}
#endif

	seq_printf(m, linux_proc_banner,
		utsname()->sysname,
#ifdef HW_VERSION
		release_buf,
#else
		utsname()->release,
#endif
		utsname()->version);
	return 0;
}

static int version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_proc_show, NULL);
}

static const struct file_operations version_proc_fops = {
	.open		= version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_version_init(void)
{
	proc_create("version", 0, NULL, &version_proc_fops);
	return 0;
}
module_init(proc_version_init);
