#!/usr/bin/env python3

import subprocess
import os
import re
import time

def listar_arquivos_pasta(pasta):
    arquivos = os.listdir(pasta)
    return arquivos

def atualizar_arquivo_world(input_path, output_path, new_robot_pose, new_target_pose):
    with open(input_path, 'r') as file:
        lines = file.readlines()

    for i, line in enumerate(lines):
        if "pioneer3dx( name \"robot0\" pose" in line:
            lines[i] = line.split("pose [")[0] + "pose [" + new_robot_pose + " ] fancysicklaser(color \"green\") )\n"
        elif "block( pose" in line and "name \"target\"" in line:
            lines[i] = line.split("pose [")[0] + "pose [" + new_target_pose + " ] name \"target\" color \"green\")\n"

    with open(output_path, 'w') as file:
        file.writelines(lines)

def atualizar_caminho_imagem_no_mundo(caminho_arquivo, nome_arquivo_escolhido):
    with open(caminho_arquivo, "r") as file:
        content = file.read()

    padrao = r'bitmap "([^"]+)"'
    novo_conteudo = re.sub(padrao, nome_arquivo_escolhido, content)

    with open(caminho_arquivo, "w") as file:
        file.write(novo_conteudo)

def atualizar_caminho_imagem_no_script(script_path, new_image_path):
    with open(script_path, 'r') as script_file:
        script_content = script_file.read()

    image_pattern = r'image_path\s*=\s*".*"'
    new_image_line = f'image_path = "{new_image_path}"'
    script_content = re.sub(image_pattern, new_image_line, script_content)

    with open(script_path, 'w') as script_file:
        script_file.write(script_content)

def atualizar_coordenadas_no_script(file_path, xstart, ystart, xend, yend):
    with open(file_path, 'r') as file1:
        content = file1.read()

    patterns = [
        (r'xstart = -?\d+\.\d+', f'xstart = {xstart}'),
        (r'ystart = -?\d+\.\d+', f'ystart = {ystart}'),
        (r'xend = -?\d+\.\d+', f'xend = {xend}'),
        (r'yend = -?\d+\.\d+', f'yend = {yend}')
    ]

    for pattern, replacement in patterns:
        content = re.sub(pattern, replacement, content)

    with open(file_path, 'w') as file1:
        file1.write(content)

def abrir_terminal_executar_comandos(comandos):
    subprocess.Popen(['gnome-terminal'])
    time.sleep(1)

    zero_linha = "cd"
    limpar_terminal = "clear"

    for comando in comandos:
        subprocess.run(['xdotool', 'type', comando])
        subprocess.run(['xdotool', 'key', 'Return'])

def main():

    user_home = os.path.expanduser("~")
    pasta = os.path.join(user_home, "catkin_ws/src/stage_controller/world/map/")
    
    if not os.path.isdir(pasta):
        print("Caminho inválido. Certifique-se de que é um diretório válido.")
        return
    
    arquivos = listar_arquivos_pasta(pasta)
    
    if not arquivos:
        print("A pasta está vazia.")
        return
    
    print("Arquivos na pasta:")
    for i, arquivo in enumerate(arquivos, start=1):
        print(f"{i}. {arquivo}")
    
    escolha = int(input("\nEscolha um número de arquivo: "))
    
    if escolha < 1 or escolha > len(arquivos):
        print("Escolha inválida. O número deve estar dentro do intervalo de arquivos listados.")
        return
    
    nome_arquivo_escolhido = os.path.join(user_home, "catkin_ws/src/stage_controller/world/map/" + arquivos[escolha - 1])
    print(f"Você escolheu o arquivo: {nome_arquivo_escolhido}")

    xstart = -12 #float(input("Digite o valor de xstart: "))
    ystart = -14 #float(input("Digite o valor de ystart: "))
    xend = 12 #float(input("Digite o valor de xend: "))
    yend = 14 #float(input("Digite o valor de yend: "))

    new_robot_pose = "{} {} 0 0".format(xstart, ystart)
    new_target_pose = "{} {} 0 0".format(xend, yend)

    
    input_file_path = os.path.join(user_home, 'catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world')
    output_file_path = os.path.join(user_home, 'catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world')

    atualizar_arquivo_world(input_file_path, output_file_path, new_robot_pose, new_target_pose)

    caminho_imagem = nome_arquivo_escolhido
    nome_arquivo_escolhido = f'bitmap "{caminho_imagem}"'
    
    caminho_arquivo = os.path.join(user_home, "catkin_ws/src/stage_controller/world/pioneer3dx-sick-ModeloCinematico_1_2.world")
    atualizar_caminho_imagem_no_mundo(caminho_arquivo, nome_arquivo_escolhido)
    
    caminho_script = os.path.join(user_home, 'catkin_ws/src/stage_controller/scripts/controle_stage_5.py')
    atualizar_caminho_imagem_no_script(caminho_script, caminho_imagem)

    file_path = os.path.join(user_home, 'catkin_ws/src/stage_controller/scripts/controle_stage_5.py')
    atualizar_coordenadas_no_script(file_path, xstart, ystart, xend, yend)

    abrir_terminal_executar_comandos([
        "cd", "cd catkin_ws", "catkin_make", ". ~/catkin_ws/devel/setup.bash", "roscore"
    ])

    abrir_terminal_executar_comandos([
        "clear", "cd", "cd catkin_ws", ". ~/catkin_ws/devel/setup.bash", "roslaunch stage_controller launcher_cinematico_1_2.launch"
    ])

    time.sleep(120)

    subprocess.run(['pkill', 'gnome-terminal'])

if __name__ == "__main__":
    main()  

