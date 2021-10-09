#!/usr/bin/env bash

# Run with file name as argument
# To plot, run in gnuplot
#   plot "data.txt" u 1:3 title 'Average' w lp, "" u 1:2 title 'Best' w lp 

printf "#Generation\tBest\t\tAverage\n" > data.txt

i=0

while IFS= read -r LINE
do
	if echo "$LINE" | grep -q "Best ="
	then
		printf "$i\t\t\t"
		COST=$(cut -c 9- <<<"$LINE")
		printf "$COST\t"
	elif echo "$LINE" | grep -q "Average"
	then
		COST=$(cut -c 12- <<<"$LINE")
		printf "$COST\n"
		((i++))
	fi
done < $1 >> data.txt

gnuplot -e "filename='data.txt'" script_gnuplot.plg
