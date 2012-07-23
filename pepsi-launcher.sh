#!/bin/bash
export LD_LIBRARY_PATH="/usr/lib64:/usr/lib:/usr/lib64/OGRE:/usr/local/lib/:/usr/local/lib/OGRE/"

BINDIR="$HOME/SODA-CD-build-desktop-Qt_4_8_1_dans_le_PATH__Syst_me__Release/"
cd $BINDIR
BINPATH="$BINDIR/SODA-CD"

for N in {500,156151612155} ; do
#for W in {12,16} ; do
for R in {1,2,3,4,5} ; do
$BINPATH -s RandomCubeSimulation -n ${N} -w 12 -x 20000 -y 6000 -z 20000 -a -t 10 -o /tmp/RCS-10sec-${N}ents-20x6x20.out${R} \
	&& echo 'Done' \
	|| echo "$BINPATH -s RandomCubeSimulation -n ${N} -w 12 -x 20000 -y 6000 -z 20000 -a -t 10 -o /tmp/RCS-10sec-${N}ents-20x6x20.out${R}" >> RCS-crashes.out;
done;
#done;
done

