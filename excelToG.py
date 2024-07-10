import pandas as pd

table = pd.read_excel(r'./trajetoria/Trajetoria.xlsx')

with open('G_trajetoria', 'w') as f:
    for i in range(len(table)):
        x = int(table['X'][i]) + 500
        y = int(table['Y'][i])
        if i+1 >= 10:
            f.write(f"N0{i+1} ")
        else:
            f.write(f"N00{i+1} ")
        f.write(f"G01 X{x} Y{y}\n")

    f.write("N051 M30")

print("Arquivo gerado com sucesso!")