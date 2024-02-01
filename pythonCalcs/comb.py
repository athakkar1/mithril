from sympy.utilities.iterables import multiset_combinations

numbers = [1.4, 2.8, 5.6, 11.2, 22.5, 45, 90, 180]

sums = []
binStrings = []

for n in range(1, 1+len(numbers)):
	for item in multiset_combinations([1.4, 2.8, 5.6, 11.2, 22.5, 45, 90, 180], n):
		added = sum(item)
		binary = [0, 0, 0, 0, 0, 0, 0, 0]
		if 1.4 in item:
			binary[7] = 1
		if 2.8 in item:
			binary[6] = 1
		if 5.6 in item:
			binary[5] = 1
		if 11.2 in item:
			binary[4] = 1
		if 22.5 in item:
			binary[3] = 1
		if 45 in item:
			binary[2] = 1
		if 90 in item:
			binary[1] = 1
		if 180 in item:
			binary[0] = 1
		binString = "".join(str(x) for x in binary)
		if not added in sums:
			sums.append(added)
			binStrings.append(int(binString, 2))

print(sums)
print(binStrings)
print(len(sums))
print(len(binStrings))
