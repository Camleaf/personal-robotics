

dataset = []
with open("main.in","r") as file:
    dataset = file.readlines();

mac,ip = 0,0
for x in dataset:
    if ":" in x or "-" in x:
        mac += 1
    else:
        ip += 1
print(mac,ip)
