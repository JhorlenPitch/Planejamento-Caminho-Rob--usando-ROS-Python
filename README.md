# Planejamento de Caminho usando RRT e Dijkstra (ROS + Python)

**Visão Geral**
Este projeto implementa um sistema de planejamento de caminho para veículos terrestres utilizando os algoritmos RRT (Rapidly-exploring Random Tree) e Dijkstra. O algoritmo RRT gera uma árvore probabilística que representa possíveis trajetórias para o veículo alcançar seu objetivo, enquanto o algoritmo Dijkstra encontra o caminho ótimo dentro do conjunto de trajetórias geradas pelo RRT. A implementação é realizada em Python, utilizando o framework ROS (Robot Operating System).

**Conteúdo do Repositório**
O repositório contém dois arquivos principais:
- **Pacote do Projeto ROS:** Este pacote contém todos os arquivos e códigos necessários para a implementação dos algoritmos RRT e Dijkstra no ROS.
- **Automação em Python:** Um script em Python foi desenvolvido para automatizar a execução do projeto, eliminando a necessidade de digitar vários comandos manualmente.
  
**Requisitos do Sistema**
- Sistema operacional Ubuntu 20.04
- ROS Noetic
- Python 
**Obs:** Garanta que o Ros esteja instalado.
  - Instale em: http://wiki.ros.org/noetic/Installation/Ubuntu
    
**Como Clonar e Executar o Projeto**
- Abra o terminal no Ubuntu 20.04.
- Navegue até o diretório do projeto.
- Clone o repositório do GitHub:
git clone https://github.com/seuusuario/nome-do-repositorio.git

**obs:** lembrese de clonar o colocar no seu workspace (Ex: catkin_ws/src/)

**Execute o script de automação em Python:**
- Mova o arquivo startrobot.py para onde desejar
- Abra o terminal e ececute: Ex: python3 startrobot.py
- 
Este script automatizará o processo de configuração e execução do projeto, tornando mais fácil o uso dos algoritmos RRT e Dijkstra no ROS.

Obrigado por explorar nosso projeto de Planejamento de Caminho usando RRT e Dijkstra com ROS e Python! Esperamos que este sistema seja útil para suas necessidades de navegação autônoma de veículos terrestres. Se você tiver alguma dúvida, sugestão ou encontrar algum problema, não hesite em entrar em contato ou abrir uma issue no GitHub.

Divirta-se explorando e contribuindo para o projeto!
