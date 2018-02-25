#!/usr/bin/python3
import re
import sys

data = ""
for line in sys.stdin:
    data += line

algorithm_parser = \
    re.compile(r'seed:(?s)\s+(\d+).*?(\d+),\s(\d+).*?(\d+),\s(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+\.\d+).*?(\d+).*?(\d+)')

rounds_info = algorithm_parser.finditer(data)

print("seed,start x,start y,goal x,goal y,nodes visited by algorithm,iterations,nodes visited by agent,visits/iteration,time(s),optimal length,optimal expanded")

for info in rounds_info:
    print(info.group(1), info.group(2), info.group(3), info.group(4), info.group(5), info.group(6), info.group(7),
          info.group(8), info.group(9), info.group(10), info.group(11), info.group(12), sep=',')
