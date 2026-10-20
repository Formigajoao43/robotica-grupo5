#!/usr/bin/env python3
"""Ponto de entrada para a sessão segura do LW0.1.

Não executar até ``config/robot.yaml`` conter os valores reais da bancada e a
sessão estar sob supervisão, com a paragem de emergência acessível.
"""

from lite6.session import safe_session


if __name__ == "__main__":
    arm = safe_session()
    try:
        print("Sessão segura preparada; nenhum movimento foi comandado.")
    finally:
        arm.disconnect()
