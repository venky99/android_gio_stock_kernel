#!/usr/bin/perl -W

use strict;
use Cwd;


my $dir = getcwd;

my $usage = "repack-bootimg.pl <kernel> <ramdisk-directory> <outfile>\n";

die $usage unless $ARGV[0] && $ARGV[1] && $ARGV[2];

chdir $ARGV[1] or die "$ARGV[1] $!";

system ("find . | cpio -o -H newc | lz4 stdin $dir/ramdisk-repack.cpio.lz4");

chdir $dir or die "$ARGV[1] $!";;

system ("./mkbootimg --kernel $ARGV[0] --ramdisk ramdisk-repack.cpio.lz4 --base 0x13600000 --pagesize 4096 -o $ARGV[2]");

unlink("ramdisk-repack.cpio.lz4") or die $!;

print "\nrepacked boot image written at $ARGV[1]-repack.img\n";
