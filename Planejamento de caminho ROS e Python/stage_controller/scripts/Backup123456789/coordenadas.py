import ast
import fileinput

def substituir_posicoes(nome_arquivo, novo_inicio, novo_fim):
    with open(nome_arquivo, 'r') as arquivo:
        conteudo = arquivo.read()

    # Procurando e substituindo a posição inicial
    inicio_antigo = conteudo.split("pose [")[1].split("]")[0]
    novo_inicio_str = ' '.join(str(i) for i in novo_inicio)
    conteudo = conteudo.replace(inicio_antigo, novo_inicio_str)

    # Procurando e substituindo a posição final
    fim_antigo = conteudo.split("pose [")[2].split("]")[0]
    novo_fim_str = ' '.join(str(i) for i in novo_fim)
    conteudo = conteudo.replace(fim_antigo, novo_fim_str)

    with open(nome_arquivo, 'w') as arquivo:
        arquivo.write(conteudo)
        
def modificar_start_end(arquivo, novo_valor_start, novo_valor_end):
    with fileinput.FileInput(arquivo, inplace=True, backup='.bak') as file:
        for line in file:
            if 'start =' in line:
                print(f'start = {novo_valor_start}')
            elif 'end =' in line:
                print(f'end = {novo_valor_end}')
            else:
                print(line, end='')
                
#Função Principal
if __name__ == "__main__":
    arquivo_world = '/home/jhorlen/catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world'
    arquivo_python = '/home/jhorlen/catkin_ws/src/stage_controller/scripts/controle_stage_5.py'
   
    # Solicita ao usuário para digitar dois valores
    start = input("Digite o inicio (x, y): ")
    
    end = input("Digite o fim (x, y): ")

    # Separa a entrada em duas partes, para obter x e y
    xi, yi = start.split()
    
    xf, yf = end.split()

    # Converte os valores para números inteiros (ou float, se necessário)
    xi = int(xi)
    yi = int(yi)
    
    xf = int(xf)
    yf = int(yf)
    
    start = (xi, yi)
    end = (xf, yf)
    
    #print(start)
    #print(end)
    
    novo_inicio = [xi, yi, 0, -180]
    novo_fim = [xf, yf, 0, 0]
    
    substituir_posicoes(arquivo_world, novo_inicio, novo_fim)
    
    modificar_start_end(arquivo_python, start, end)
################################

    

