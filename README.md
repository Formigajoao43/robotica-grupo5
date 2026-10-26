# Robótica — Laboratório 0 | Parte 2

**Mestrado em Engenharia Eletrotécnica e de Computadores**
Universidade de Coimbra — Ano letivo 2026/2027

## 1. Sobre o projeto

Este repositório reúne os ficheiros e a configuração do ambiente de desenvolvimento utilizado no Laboratório 0 — Parte 2 da unidade curricular de Robótica.

O objetivo deste trabalho foi preparar um ambiente que permita desenvolver, testar e simular aplicações para o braço robótico **UFACTORY Lite 6**, sem depender exclusivamente do robô físico.

Para isso, configurámos duas componentes: um ambiente Python, com acesso ao SDK da UFACTORY, e um ambiente de simulação baseado em ROS 2, MoveIt 2 e Gazebo.

## 2. Ambiente utilizado

O trabalho foi realizado num **MacBook Air com processador Apple M2**, utilizando o macOS.

As principais ferramentas utilizadas foram:

- **Python 3.13.7** — linguagem utilizada no projeto.
- **VS Code** — editor de código.
- **xArm Python SDK 1.18.4** — comunicação com o controlador UFACTORY.
- **Git** — controlo de versões.
- **Docker Desktop** — execução do ambiente ROS 2.
- **ROS 2 Jazzy** — comunicação entre os componentes do sistema.
- **MoveIt 2** — planeamento de movimentos.
- **RViz 2** — visualização do robô e das suas configurações.
- **Gazebo Harmonic** — simulação física do robô.

Optámos por utilizar Docker para o ambiente ROS 2, evitando a necessidade de instalar diretamente todas as dependências no macOS.

## 3. Ambiente Python

Foi criado um ambiente virtual para manter as bibliotecas do projeto separadas das restantes instalações do computador.

Para preparar o ambiente, abrir o Terminal na pasta do projeto e executar:

```bash
python3 -m venv ~/venvs/robotica
source ~/venvs/robotica/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

No VS Code, deve ser selecionado o interpretador:

`~/venvs/robotica/bin/python`

Recomendamos também as extensões **Python** e **Pylance**, ambas da Microsoft.

### Estrutura do projeto

```text
robotica-grupoNN/
├── config/
│   └── robot.yaml
├── lite6/
│   ├── __init__.py
│   ├── session.py
│   ├── kinematics.py
│   ├── ik.py
│   ├── velocity.py
│   ├── trajectory.py
│   └── game.py
├── scripts/
│   ├── safe_session.py
│   └── log_joints.py
├── tests/
│   └── test_environment.py
├── logs/
├── requirements.txt
└── README.md
```

Os ficheiros principais utilizados nesta fase são:

- `config/robot.yaml` — guarda os parâmetros de configuração do robô.
- `lite6/session.py` — contém as funções de leitura da configuração e preparação de uma sessão com o Lite 6.
- `scripts/safe_session.py` — permite iniciar uma sessão através do módulo anterior.
- `tests/test_environment.py` — contém verificações automáticas que não exigem ligação ao robô.

Alguns ficheiros da estrutura encontram-se ainda sem implementação, estando reservados para utilização posterior.

### Testes do ambiente Python

Para executar os testes automáticos:

```bash
python -m pytest -v tests
```

Na validação inicial foram executados **3 testes, todos concluídos com sucesso**.

Foi também confirmada a importação do módulo `safe_session` e verificada a ausência de endereços IP fixos no código Python das pastas `lite6` e `scripts`.

A ligação efetiva ao controlador ainda depende dos parâmetros da bancada e da respetiva validação de segurança.

## 4. Ambiente ROS 2 e simulação

Para trabalhar com o ROS 2 no macOS, utilizámos a adaptação para Mac do ambiente disponibilizado para a unidade curricular:

https://github.com/ritaisabelmorais/robotica

Este ambiente é mantido numa pasta separada do projeto Python.

### Preparação

Caso o ambiente Docker ainda não esteja instalado, deve ser seguido o README do repositório indicado acima.

Para obter o projeto ROS 2:

```bash
cd ~/Robotica
git clone -b mac https://github.com/ritaisabelmorais/robotica.git
cd robotica
cp .env.example .env
```

Antes de utilizar o ambiente na rede do laboratório, é necessário configurar no ficheiro `.env` o `ROS_DOMAIN_ID` atribuído ao grupo.

Para construir e iniciar o ambiente:

```bash
./lab.sh build
./lab.sh up
./lab.sh doctor
```

O comando `build` só precisa de ser repetido quando houver alterações que exijam reconstruir a imagem.

Na configuração utilizada, a construção foi concluída e o comando `doctor` terminou com a mensagem **All checks passed**.

### Utilização

Depois de iniciar o Docker, é possível entrar no ambiente ROS 2 através de:

```bash
./lab.sh shell
```

No macOS, a interface gráfica pode ser aberta no navegador:

http://localhost:6080/

Esta interface permite visualizar o RViz e o Gazebo sem instalar os programas diretamente no sistema operativo.

Para desligar o ambiente:

```bash
./lab.sh down
```

Os ficheiros de trabalho mantidos nas pastas partilhadas não são apagados ao desligar os contentores.

## 5. Verificações realizadas

Para confirmar o funcionamento do ambiente, foram realizados testes de comunicação ROS 2, identificação dos pacotes UFACTORY, visualização do Lite 6 e simulação.

Até ao momento, conseguimos:

- Confirmar a comunicação entre os nós `talker` e `listener`.
- Identificar os pacotes ROS 2 necessários para o Lite 6.
- Abrir o modelo do robô no RViz com o MoveIt 2.
- Gerar o diagrama dos referenciais do robô através de TF.
- Consultar transformações entre a base e a extremidade do braço.
- Executar uma simulação básica no Gazebo.
- Planear e executar movimentos do Lite 6 no ambiente de simulação.

Durante o encerramento de uma sessão do MoveIt 2 foi observado um erro associado ao processo `move_group`. Apesar disso, a visualização e a interação com o robô tinham funcionado durante o teste.

Ainda falta completar alguns registos das transformações e a validação com o simulador de firmware da UFACTORY.

Os resultados completos, incluindo as medições e eventuais erros, serão apresentados no relatório laboratorial.

## 6. Configuração e segurança

Os parâmetros do robô encontram-se no ficheiro `config/robot.yaml`, permitindo alterar a configuração sem modificar diretamente o código Python.

Os valores que dependem da bancada, como o endereço IP do controlador, a posição inicial e os limites do espaço de trabalho, devem ser confirmados antes da utilização do robô físico.

O script `safe_session.py` foi preparado para impedir a abertura de uma sessão quando faltam parâmetros essenciais. No entanto, a sua validação em hardware ainda está pendente.

**Não devem ser executados movimentos no robô físico sem a configuração adequada, validação prévia em simulação e autorização do docente.**

## 7. Controlo de versões

O projeto utiliza Git para registar as alterações realizadas ao longo do desenvolvimento.

Os ficheiros de configuração, o código Python, os testes e a documentação são mantidos no mesmo repositório, permitindo acompanhar a evolução do trabalho e reproduzir os resultados.

As versões destinadas à entrega serão identificadas através das etiquetas Git indicadas pelo professor.

---

*Laboratório 0 — Parte 2 | Robótica | Universidade de Coimbra*