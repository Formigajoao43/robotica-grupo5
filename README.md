# Robótica — grupo NN

Projeto Python do grupo para a UC Robótica (02000537), MEEC, 2026/2027. Este
repositório é distinto do workspace ROS 2/Docker em `~/Robotica/robotica`.

## Estado e segurança

Os campos dependentes da bancada em `config/robot.yaml` estão intencionalmente
por preencher: identificador do grupo, IP, carga e offset TCP, pose home,
fronteira TCP e pose do tabuleiro. Não substituir estes valores por exemplos.
`safe_session` recusa abrir uma ligação enquanto IP e fronteira TCP não estiverem
configurados.

O LW0.1 exige simulação antes de hardware, modo reduzido, fronteira TCP e um
membro junto à paragem de emergência. Nenhum ficheiro deste repositório comanda
movimentos automaticamente. `scripts/log_joints.py` permanece pendente até o
modo manual/carga da bancada serem validados com o docente.

## Ambiente intermédio — LW0.2 B.1–B.5

```bash
source ~/venvs/robotica/bin/activate
python -m pip install -r requirements.txt
python -m pytest -v tests
```

Abra esta pasta no VS Code. A configuração seleciona
`~/venvs/robotica/bin/python`, ativa-o no terminal integrado, permite depurar o
ficheiro atual e executar testes `pytest` locais. As extensões requeridas são
Python e Pylance da Microsoft.

## Estrutura

- `config/robot.yaml`: parâmetros da bancada, sem valores inventados.
- `lite6/session.py`: YAML e sequência segura do LW0.1.
- `lite6/kinematics.py`: marcador para a interface do grupo; não contém DH.
- `lite6/ik.py`, `velocity.py`, `trajectory.py`, `game.py`: marcadores para
  LW2, LW3, LW4 e Desafio Final.
- `scripts/`: um ponto de entrada por experiência; hardware requer validação.
- `tests/`: testes locais sem rede ou hardware.
- `logs/`: resultados reais; o conteúdo é ignorado pelo Git.

## LW1 — dependência pendente

O enunciado exige o pacote oficial ROS 2 `src/lite6_kinematics` e permite editar
somente `forward_kinematics.py` (apenas `numpy` e `math`). Esse template não foi
encontrado no workspace local em 20/10/2026. Não foi criado um substituto nem
foram introduzidos parâmetros DH. Quando o pacote oficial for disponibilizado,
copie-o para `~/Robotica/robotica/src/`, implemente apenas o ficheiro autorizado
e execute primeiro os testes unitários fornecidos pelo docente.

## Docker/ROS 2

O Docker/ROS 2 usa o repositório separado `~/Robotica/robotica`. O respetivo
README pede pelo menos 8 GB de memória e 60 GB de disco para a primeira imagem;
este Mac tinha 4 GB atribuídos ao Docker e cerca de 31 GiB livres na verificação
de 20/10/2026. Por isso não foi iniciado `./lab.sh build`. Não usar
`ROS_DOMAIN_ID=0` em laboratório partilhado: aguardar o identificador atribuído.

## Git e entregas

Não publicar sem autorização. Criar as etiquetas `delivery-1`, `delivery-2` e
`delivery-3` apenas nas versões efetivamente entregues.
