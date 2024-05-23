import fileinput

# Caminhos dos arquivos
input_file_path = '/home/jhorlen/catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world'
output_file_path = '/home/jhorlen/catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world'

arquivo_alvo = '/home/jhorlen/catkin_ws/src/stage_controller/scripts/controle_stage_5.py'

xstart = float(input("Digite o valor de xstart: "))
ystart = float(input("Digite o valor de ystart: "))

print("\n")

xend = float(input("Digite o valor de xstart: "))
yend = float(input("Digite o valor de ystart: "))

new_robot_pose = "{} {} 0 0".format(xstart, ystart)
new_target_pose = "{} {} 0 0".format(xend, yend)

# Abrir o arquivo .world para leitura
with open(input_file_path, 'r') as file:
    lines = file.readlines()

# Alterar as posições do robô e do objeto
for i, line in enumerate(lines):
    if "pioneer3dx( name \"robot0\" pose" in line:
        lines[i] = line.split("pose [")[0] + "pose [" + new_robot_pose + " ] fancysicklaser(color \"green\") )\n"
    elif "block( pose" in line and "name \"target\"" in line:
        lines[i] = line.split("pose [")[0] + "pose [" + new_target_pose + " ] name \"target\" color \"green\")\n"

# Abrir o arquivo .world para escrita, sobrescrevendo o conteúdo original
with open(output_file_path, 'w') as file:
    file.writelines(lines)
    
novo_start = xstart
novo_ystart = ystart

novo_xend = xend
novo_yend = yend
    
# Função para substituir as variáveis no arquivo
def substituir_variaveis_linha(linha):
    linha = linha.replace("xstart =", f"xstart = {novo_xstart}")
    linha = linha.replace("ystart =", f"ystart = {novo_ystart}")
    linha = linha.replace("xend =", f"xend = {novo_xend}")
    linha = linha.replace("yend =", f"yend = {novo_yend}")
    return linha

# Abrir o arquivo em modo de leitura e escrita
with fileinput.FileInput(arquivo_alvo, inplace=True) as arquivo:
    for linha in arquivo:
        linha_modificada = substituir_variaveis_linha(linha)
        print(linha_modificada, end="")
