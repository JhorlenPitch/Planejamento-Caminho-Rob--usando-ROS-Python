import fileinput

# Defina os novos valores das variáveis
novo_xstart = 10
novo_ystart = 20
novo_xend = 100
novo_yend = 200

# Arquivo que você quer modificar
arquivo_alvo = '/home/jhorlen/catkin_ws/src/stage_controller/scripts/controle_stage_5.py'

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
