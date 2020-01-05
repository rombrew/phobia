#!/home/amaora/util/gp
# vi: ft=conf

chunk 20

load 0 10000 text "/dev/rfcomm0"
mkpages -1

group 0 -1
deflabel 0 "Time (s)"
defscale 0 0.05 0

