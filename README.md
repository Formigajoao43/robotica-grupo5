
# Robótica — Laboratório 0 | Parte 2

**Mestrado em Engenharia Eletrotécnica e de Computadores**  
Universidade de Coimbra — Ano letivo 2026/2027

## 1. Sobre o projeto

Este repositório contém o código, as configurações e a documentação do Laboratório 0 — Parte 2 da unidade curricular de Robótica.

O objetivo foi preparar um ambiente para desenvolver e testar aplicações para o braço robótico UFACTORY Lite 6, utilizando Python, ROS 2 e ferramentas de simulação.

Foram preparados dois ambientes:
- Python e xArm SDK, para comunicação com o controlador.
- ROS 2, MoveIt 2 e Gazebo, para visualização, planeamento e simulação.

## 2. Ambiente utilizado

O trabalho foi realizado num MacBook Air com processador Apple M2 e macOS.

Ferramentas utilizadas:

- Python 3.13.7
- VS Code
- xArm Python SDK 1.18.4
- Git e GitHub
- Docker Desktop
- ROS 2 Jazzy
- MoveIt 2
- RViz 2
- Gazebo Harmonic

Para o nível avançado, escolhemos a **Via 1 — Docker**, utilizando a adaptação para macOS indicada no enunciado.

O ROS 2 e o Gazebo foram executados em Docker. O ambiente Python e o SDK foram utilizados diretamente no macOS.

## 3. Preparação do ambiente Python

### 3.1. Obter o projeto

O repositório do grupo está disponível em:

https://github.com/Formigajoao43/robotica-grupoNN

Para o obter numa instalação nova:

```bash
mkdir -p ~/Robotica
cd ~/Robotica
git clone https://github.com/Formigajoao43/robotica-grupoNN.git
cd robotica-grupoNN
```

Se o projeto já estiver instalado, basta abrir a respetiva pasta.

### 3.2. Criar o ambiente virtual

Na pasta do projeto:

```bash
python3 -m venv ~/venvs/robotica
source ~/venvs/robotica/bin/activate

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

No VS Code, selecionar o interpretador:

`~/venvs/robotica/bin/python`

Foram utilizadas as extensões Python e Pylance.

### 3.3. Estrutura do projeto

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
│   ├── home.py
│   ├── log_joints.py
│   └── teste8_firmware.py
├── tests/
│   └── test_environment.py
├── logs/
├── requirements.txt
└── README.md
```

Nesta fase, os ficheiros principais são:

- `lite6/session.py`: leitura da configuração e preparação da sessão segura.
- `scripts/safe_session.py`: ponto de entrada para a sessão segura.
- `scripts/home.py`: comando e verificação da posição home.
- `scripts/teste8_firmware.py`: leitura de juntas, pose e cinemática direta no firmware.
- `config/robot.yaml`: parâmetros do robô e limites de segurança.
- `tests/test_environment.py`: testes automáticos do ambiente.

Os módulos destinados aos laboratórios seguintes podem ainda não ter implementação.

## 4. Validação do ambiente Python — B.5

Para executar os testes automáticos:

```bash
cd ~/Robotica/robotica-grupoNN
source ~/venvs/robotica/bin/activate

python -m pytest -v tests
```

Na validação inicial, os três testes terminaram com sucesso.

Para confirmar a importação do módulo:

```bash
python -c "from lite6.session import safe_session; print('Importação OK')"
```

Para verificar a ausência de endereços IP fixos da gama 192.168 no código:

```bash
grep -R -n "192\.168" lite6 scripts
```

Se não aparecerem correspondências, essa sequência não foi encontrada nos ficheiros analisados.

A função `safe_session()` utiliza parâmetros de `config/robot.yaml` e bloqueia a abertura da sessão quando falta uma configuração essencial, como a fronteira TCP.

A ligação ao simulador de firmware foi também testada, conforme descrito na secção 7.

**Nota:** a ligação ao simulador não substitui a validação de segurança necessária antes de utilizar o braço físico.

## 5. Ambiente ROS 2 e Gazebo

### 5.1. Instalação

Foi utilizado o repositório adaptado para macOS:

https://github.com/ritaisabelmorais/robotica

Este ambiente é mantido separadamente do projeto Python.

Para uma instalação nova:

```bash
cd ~/Robotica

git clone -b mac https://github.com/ritaisabelmorais/robotica.git

cd robotica
cp .env.example .env
```

Antes de utilizar o ROS 2 numa rede partilhada, é necessário colocar no `.env` o `ROS_DOMAIN_ID` atribuído ao grupo pelo docente.

Durante os testes locais isolados foi utilizado temporariamente o identificador `0`. Esse identificador não deve ser utilizado pelo grupo na rede do laboratório, pois está reservado à equipa docente.

### 5.2. Iniciar o ambiente

```bash
cd ~/Robotica/robotica

./lab.sh build
./lab.sh up
./lab.sh doctor
```

A construção foi concluída e o diagnóstico terminou com a mensagem:

`All checks passed`

O comando `build` não precisa de ser repetido em todas as sessões.

Para abrir uma shell ROS 2:

```bash
./lab.sh shell
```

No macOS, o ambiente gráfico está disponível em:

http://localhost:6080/

Para desligar os contentores:

```bash
./lab.sh down
```

### 5.3. Repetir o Teste 1 — Comunicação ROS 2

Com o ambiente Docker iniciado:

```bash
./lab.sh shell
```

Dentro do contentor:

```bash
lab test 1
```

O teste utiliza os nós `talker` e `listener`.

O resultado esperado é a publicação e receção de mensagens `Hello World`, com os números das mensagens a corresponder.

Este teste passou na validação inicial.

### 5.4. Repetir o Teste 3 — RViz e MoveIt 2

Dentro do contentor ROS 2:

```bash
lab test 3
```

Abrir o ambiente gráfico em:

http://localhost:6080/

Confirmar que:
- O modelo Lite 6 aparece no RViz.
- O painel MotionPlanning está disponível.
- É possível interagir com o modelo simulado.

O teste passou na validação inicial.

Foi observado um erro associado ao processo `move_group` durante o encerramento de uma sessão. Esse problema não impediu a visualização e utilização inicial do robô.

## 6. Resultados dos Testes 1–8

Foram realizados os testes do nível avançado previstos no §C.5 do enunciado.

| Teste | Descrição | Resultado |
|---|---|---|
| 1 | Comunicação entre talker e listener | Passa |
| 2 | Identificação dos pacotes UFACTORY | Passa — 10 pacotes |
| 3 | Modelo Lite 6 no RViz | Passa |
| 4 | Diagrama dos referenciais TF | Passa |
| 5 | Leitura das transformações | Matrizes obtidas; associação dos ângulos a confirmar |
| 6 | Simulação de formas no Gazebo | Passa |
| 7 | Lite 6 no Gazebo com MoveIt | Passa |
| 8 | Comunicação com firmware UFACTORY | Passa |

No Teste 4 foi obtida a cadeia cinemática do braço, incluindo os referenciais `link_base` e `link_eef`.

No Teste 5 foram recolhidas duas matrizes de transformação. A correspondência exata entre cada matriz e os ângulos das juntas deve ser confirmada na documentação final.

No Teste 7 foi possível executar movimentos do Lite 6 através do MoveIt e observar o robô no Gazebo.

Os resultados completos e as evidências encontram-se no relatório laboratorial.

## 7. Simulador de firmware UFACTORY

### 7.1. Preparação

Para testar o xArm SDK sem utilizar o braço físico, foi utilizado o simulador de firmware da UFACTORY através de Docker.

Na primeira utilização:

```bash
docker pull danielwang123321/uf-ubuntu-docker
```

Criar o contentor:

```bash
docker run -it \
  --name uf_software \
  -p 18333:18333 \
  -p 502:502 \
  -p 503:503 \
  -p 504:504 \
  -p 30000:30000 \
  -p 30001:30001 \
  -p 30002:30002 \
  -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker
```

Se o contentor já existir mas estiver parado:

```bash
docker start -ai uf_software
```

Dentro do contentor, iniciar o firmware com o perfil Lite 6:

```bash
/xarm_scripts/xarm_start.sh 6 9
```

Manter o contentor em execução.

Abrir o UFACTORY Studio no navegador:

http://127.0.0.1:18333/

O endereço `127.0.0.1` corresponde ao próprio Mac. As portas publicadas pelo Docker permitem comunicar com o controlador virtual.

### 7.2. Repetir o Teste 8

Noutro terminal do Mac:

```bash
cd ~/Robotica/robotica-grupoNN
source ~/venvs/robotica/bin/activate
```

Confirmar que `config/robot.yaml` contém:

```yaml
robot:
  ip: "127.0.0.1"
```

Não é necessário substituir o restante conteúdo do YAML.

Executar:

```bash
python scripts/teste8_firmware.py
```

O script consulta:
- Os ângulos das juntas.
- A pose cartesiana do robô.
- A cinemática direta da Pose A, utilizando `get_forward_kinematics()`.

Na configuração inicial, foram obtidos:

```text
Juntas: [0, 0, 0, 0, 0, 0]

Pose:
X = 87 mm
Y = 0 mm
Z = 154,199997 mm

Cinemática direta:
X = 87 mm
Y ≈ 0 mm
Z ≈ 154,200 mm
```

A comparação com a matriz TF da Pose A deu uma diferença aproximada de `0,20 mm`.

Este resultado utiliza os valores arredondados apresentados pelo `tf2`.

### 7.3. Validação de safe_session

Foi testada a função `safe_session()` diretamente contra o simulador de firmware.

Como a configuração original da bancada ainda não está aprovada, utilizou-se um ficheiro YAML temporário, sem alterar `config/robot.yaml`.

Para repetir este teste, com o simulador em execução:

```bash
python - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
import yaml

from lite6.session import load_robot_config, safe_session

config = load_robot_config()

if config["robot"]["ip"] != "127.0.0.1":
    raise RuntimeError("Teste apenas no simulador local.")

# Fronteira de ensaio, não aprovada para hardware.
config["safety"]["tcp_boundary_mm"] = [
    300, -300, 300, -300, 500, 0
]

with TemporaryDirectory() as pasta:
    ficheiro = Path(pasta) / "robot_sim.yaml"
    ficheiro.write_text(
        yaml.safe_dump(config),
        encoding="utf-8"
    )

    arm = None
    try:
        arm = safe_session(ficheiro)

        print("Sessão preparada no simulador.")

        code, angles = arm.get_servo_angle()

        print("Código de leitura:", code)
        print("Juntas:", angles[:6])

    finally:
        if arm is not None:
            arm.disconnect()
PY
```

Na execução realizada, a sessão foi preparada sem erros e a leitura das juntas devolveu código `0`.

Esta verificação configura o controlador virtual, mas não comanda qualquer movimento.

### 7.4. Teste de movimento home no simulador

Foi também realizado um movimento de ensaio com a seguinte configuração:

```text
Home de ensaio: [5, 0, 0, 0, 0, 0] graus
Velocidade comandada: 20 graus/s
```

A fronteira TCP utilizada foi:

```text
[300, -300, 300, -300, 500, 0] mm
```

Estes valores destinam-se apenas ao simulador e não são parâmetros aprovados para o braço físico.

Para repetir o teste no simulador, depois de confirmar que `scripts/home.py` aceita o argumento `config_path`:

```bash
python - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
import yaml

from lite6.session import load_robot_config
from scripts.home import main

config = load_robot_config()

if config["robot"]["ip"] != "127.0.0.1":
    raise RuntimeError("Teste apenas no simulador local.")

config["safety"]["tcp_boundary_mm"] = [
    300, -300, 300, -300, 500, 0
]

config["home"]["joints_deg"] = [
    5, 0, 0, 0, 0, 0
]

config["home"]["speed_deg_s"] = 20

with TemporaryDirectory() as pasta:
    ficheiro = Path(pasta) / "robot_sim.yaml"
    ficheiro.write_text(
        yaml.safe_dump(config),
        encoding="utf-8"
    )

    main(ficheiro)
PY
```

Na execução realizada, o SDK devolveu:

```text
Home definida:
[5, 0, 0, 0, 0, 0]

Ângulos finais:
[4.999973, 0, 0, 0, 0, 0]
```

O maior erro absoluto entre as juntas comandadas e as juntas lidas foi:

`0,000027°`

O movimento foi também confirmado no UFACTORY Studio.

O valor obtido corresponde a uma leitura do firmware virtual, não a uma medição da precisão do braço físico.

### 7.5. Verificação do modo reduzido

Após a preparação da sessão, foi consultado o estado do modo reduzido com `get_reduced_states()`.

Para repetir a consulta:

```bash
python - <<'PY'
from xarm.wrapper import XArmAPI

arm = XArmAPI(
    "127.0.0.1",
    is_radian=False,
    check_joint_limit=False
)

try:
    code, states = arm.get_reduced_states()

    print("Código:", code)
    print("Estado:", states)

finally:
    arm.disconnect()
PY
```

Foram confirmados no firmware:

| Parâmetro | Resultado |
|---|---|
| Modo reduzido | Ativo |
| Velocidade máxima das juntas | 54°/s |
| Velocidade máxima TCP | 150 mm/s |
| Fronteira TCP | Configuração de ensaio aceite |

A velocidade de 20°/s foi comandada no teste `home`, mas a velocidade efetiva durante o movimento não foi medida.

### 7.6. Problema encontrado no SDK

Durante a primeira tentativa de executar `home`, foi identificado um erro na validação local dos limites das juntas, causado por um número de série incompleto disponibilizado pelo simulador.

A execução gerou um `ValueError` dentro do SDK.

Para os testes locais com o simulador foi utilizada a opção:

```python
check_joint_limit=False
```

Essa opção é aplicada apenas quando o IP corresponde a `127.0.0.1`. Para os restantes endereços, a verificação local dos limites mantém-se ativa.

A alteração permitiu executar o movimento de ensaio, mantendo as restantes verificações implementadas em `safe_session()`.

## 8. Segurança

Os parâmetros do robô são mantidos em:

`config/robot.yaml`

A configuração utilizada no projeto inclui:

```yaml
robot:
  ip: "127.0.0.1"
  model: lite6
  sdk_units:
    length: mm
    angle: deg

safety:
  reduced_max_joint_speed_deg_s: 54
  reduced_max_tcp_speed_mm_s: 150
  tcp_boundary_mm: null
  tcp_load: null

home:
  joints_deg: null
  speed_deg_s: 20
```

Os restantes campos do ficheiro original devem ser mantidos.

Os valores `null` indicam parâmetros ainda não definidos ou aprovados.

A função `safe_session()` bloqueia a abertura da sessão quando faltam parâmetros essenciais.

Foram realizados testes locais que confirmaram:
- Bloqueio quando a fronteira TCP está em falta.
- Bloqueio de `home` quando os ângulos não estão definidos.
- Bloqueio de velocidades superiores ao limite configurado.
- Interrupção e desconexão perante uma falha simulada do SDK.
- Preparação da sessão no firmware virtual.

A posição `home` e a fronteira TCP utilizadas nos ensaios não são válidas para utilização física sem verificação e aprovação.

**Não executar movimentos no robô físico sem autorização do docente, parâmetros da bancada validados e condições de segurança asseguradas.**

## 9. Verificação de reprodutibilidade

O ponto D.4 do enunciado exige que o ambiente possa ser reproduzido a partir de uma shell limpa, seguindo apenas este README.

O procedimento é:

1. Obter o repositório e preparar o ambiente Python (§3).
2. Executar a validação B.5 (§4).
3. Iniciar o ambiente ROS 2 e executar o Teste 1 (§5.3).
4. Executar o Teste 3 e confirmar o Lite 6 no RViz (§5.4).
5. Iniciar o simulador de firmware (§7.1).
6. Executar o Teste 8 através do SDK (§7.2).

Os comandos e os resultados iniciais estão documentados nas secções anteriores.

A repetição integral deste procedimento a partir de uma shell limpa deve ser registada separadamente. Os resultados anteriores não são apresentados como prova de que essa repetição completa já foi efetuada.

## 10. Controlo de versões

O projeto utiliza Git e GitHub para permitir o desenvolvimento em grupo e registar as alterações realizadas.

Repositório:

https://github.com/Formigajoao43/robotica-grupoNN

Para verificar o estado local:

```bash
git status
```

Para consultar os commits:

```bash
git log --oneline
```

Para consultar as etiquetas existentes:

```bash
git tag -l
```

As etiquetas destinadas à entrega devem respeitar as indicações do docente.

Antes de efetuar alterações partilhadas, é necessário confirmar que não existem modificações locais ou conflitos com o trabalho de outro elemento do grupo.

---

*Laboratório 0 — Parte 2 | Robótica | Universidade de Coimbra*
