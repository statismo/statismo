#!/bin/sh

#curl http://www.boost.org/patches/1_55_0/001-log_fix_dump_avx2.patch | patch -p1

if [ `echo "x$OS" | grep -c Windows` -eq 1 ]; then
	# Boost ships some .bat files with Unix line endings, which confuses
	# cmd.exe in some circumstances, and causes an
	# "input line is too long" error, resulting in a failed build.
	#
	# This is an ugly hack to work around that issue.
	find . -name "*.bat" -print0 | xargs -0 -I{} unix2dos {}
fi

