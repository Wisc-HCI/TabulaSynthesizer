#!/bin/bash

# Author: David Porfirio <dporfirio@wisc.edu>


RED="\e[1;31m"
YELLOW="\e[1;33m"
GREEN="\e[1;32m"
WHITE="\e[0m"

verbose="False"
reset_oracle=""
while getopts "rv" arg; do
  case $arg in
    r)
      reset_oracle="reset" 
      ;;
    v)
      verbose="True"
      ;;
  esac
done

FOLDERS=../test_cases/*

count=0
corr_count=0

for fold in $FOLDERS; do
	test_category=`echo $fold | sed 's:.*/::'`
	echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
	echo "Running tests within" $test_category "folder"
	echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
	fold="${fold}/*"
  for f in $fold; do
		if [[ $reset_oracle == "reset" ]]; then
			python3 pipeline.py -f $f -o 1> out.txt 2> fail.txt
			errmsg=`cat fail.txt`
			if [[ $errmsg != "" ]]; then
				cat fail.txt
				bn=`basename $f`
				echo ERROR $bn
				echo Please fix and rerun ./test_bash.sh -r
				exit 1
			fi
		else
			start=`date +%s.%N`
			python3 pipeline.py -f $f 1> out.txt 2> fail.txt
			end=`date +%s.%N`
			bn=`basename $f`
			ofile="./test_files/oracle/${bn}.txt"
			tfile="./test_files/temp/${bn}.txt"
			d=`diff $ofile $tfile`
			errmsg=`cat fail.txt`
			if [[ $errmsg != "" ]]; then
				cat fail.txt
				bn=`basename $f`
				echo ERROR $bn
				echo Stopping tests prematurely.
				exit 1
			elif [[ $d == "" ]]; then
				runtime=$(echo "$end - $start" | bc -l)
				printf "${GREEN}PASSED${WHITE} %s\n" "test $bn ($runtime seconds)"
				corr_count=$(($corr_count+1))	
			else
				printf "${RED}FAILED${WHITE} %s\n" "test $bn"
				cat out.txt >> err.txt
			fi
			if [[ $verbose == "True" ]]; then
				echo "======verbose output========"
				cat "./test_files/temp/${bn}.txt"
				echo
				echo
			fi
			count=$(($count+1))
		fi
  done
  echo
  echo
done

color=$GREEN
if (( $corr_count == 0 )); then
  color=$RED
elif (( $corr_count != $count )); then
	color=$YELLOW
fi
printf "${color}%s${WHITE}\n" "RESULT: $corr_count out of $count tests passed."

if [[ $corr_count == $count ]]; then
	echo Great job!
fi
