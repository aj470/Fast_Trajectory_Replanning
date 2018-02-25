#!/usr/bin/python3
import re
import sys

data = ""
for line in sys.stdin:
    data += line

algorithm_parser = \
    re.compile(r'seed:(?s)\s+(\d+).*?(\d+),\s(\d+).*?(\d+),\s(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+).*?(\d+\.\d+)')

memory_parser = re.compile(r'(\d+(?:\.\d+)?)\s(MB|KB|B)')

rounds_info = algorithm_parser.finditer(data)
memory_info = memory_parser.finditer(data)

print("seed,start x,start y,goal x,goal y,nodes visited by algorithm,iterations,nodes visited by agent,visits/iteration,optimal route length,optimal visits,time(s)")

for info in rounds_info:
    print(info.group(1), info.group(2), info.group(3), info.group(4), info.group(5), info.group(6), info.group(7),
          info.group(8), info.group(9), info.group(10), info.group(11), info.group(12), sep=',')

sum = 0
for info in memory_info:
    factors = {"B": 1, "KB": 1024, "MB": 1024*1024}
    factor = factors[info.group(2)]
    sum += float(info.group(1)) * factor

print(sum)