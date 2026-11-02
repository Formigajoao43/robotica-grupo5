
"""Ponto de entrada para a sessão segura do Lite 6."""

from lite6.session import safe_session


def main():
    arm = safe_session()

    try:
        print("Sessão segura preparada.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
