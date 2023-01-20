import shelve
filename='/tmp/shelve.out'

my_shelf = shelve.open(filename)
for key in my_shelf:
    globals()[key]=my_shelf[key]
my_shelf.close()

print(T)
# Hiya
print(val)
# [1, 2, 3]