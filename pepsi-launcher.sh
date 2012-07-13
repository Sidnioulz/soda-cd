#!/bin/bash
export LD_LIBRARY_PATH="/usr/lib64:/usr/lib:/usr/lib64/OGRE:/usr/local/lib/:/usr/local/lib/OGRE/"

BINDIR="$HOME/Code/Build-SODA-CD/"
cd $BINDIR
BINPATH="$BINDIR/SODA-CD"

for N in {10,50,100,200,400,800,1500,2000,3500,5000,7500,10000,20000,30000,40000} ; do
`$BINPATH -s RandomCubeSimulation -n $N -x 5000 -y 2000 -z 5000 -a -t 10 -o /tmp/RCS-10sec-$Nents-5x2x5.out` \
	&& echo 'Done' \
	|| echo "$BINPATH -s RandomCubeSimulation -n $N -x 5000 -y 2000 -z 5000 -a -t 10 -o /tmp/RCS-10sec-$Nents-5x2x5.out" >> RCS-crashes.out;
done

