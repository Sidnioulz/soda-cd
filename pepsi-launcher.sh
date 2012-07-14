#!/bin/bash
export LD_LIBRARY_PATH="/usr/lib64:/usr/lib:/usr/lib64/OGRE:/usr/local/lib/:/usr/local/lib/OGRE/"

BINDIR="$HOME/SODA-CD-build-desktop-Qt_4_8_1_dans_le_PATH__Syst_me__Release/"
cd $BINDIR
BINPATH="$BINDIR/SODA-CD"

#for N in {400,800,1500,2000,3500,5000,7500,10000,20000,30000,40000} ; do
for R in {1,2,3,4,5} ; do
for N in {1000,2000} ; do
for W in {2,4,6,8,10,12,16,20,24,28,32,40,48,56,64} ; do
$BINPATH -s RandomCubeSimulation -n $N -w $W -x 10000 -y 10000 -z 10000 -a -t 10 -o /tmp/RCS-10sec-${W}worlds-${N}ents-10x10x10.out${R} \
	&& echo 'Done' \
	|| echo "$BINPATH -s RandomCubeSimulation -n $N -w $W -x 10000 -y 10000 -z 10000 -a -t 10 -o /tmp/RCS-10sec-${W}worlds-${N}ents-10x10x10.out${R}" >> RCS-crashes.out;
done;
done;
done

