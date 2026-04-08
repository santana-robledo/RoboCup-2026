import time
import Control

# Configuración del puerto
PORT = "/dev/ttyACM0"  # Cambia según tu puerto

def send_manual_command(Ux=0.0, Uy=0.0, Ut=0.0, Patada=0, Cilindro=0):
    """
    Envía comando al robot en formato:
    M,Ux,Uy,Ut,Patada,Cilindro,N,0,0
    """
    Control.send(Ux, Uy, Ut, Patada, Cilindro, "N", force=True)

def parse_input(line):
    """
    Convierte la entrada de usuario a floats e ints.
    Formato esperado: Ux,Uy,Ut,Patada,Cilindro
    """
    try:
        parts = line.strip().split(",")
        if len(parts) != 5:
            return None
        Ux = float(parts[0])
        Uy = float(parts[1])
        Ut = float(parts[2])
        Patada = int(parts[3])
        Cilindro = int(parts[4])
        return Ux, Uy, Ut, Patada, Cilindro
    except ValueError:
        return None

def main():
    # Conectar a Arduino
    Control.connect(PORT)
    time.sleep(2)
    Control.init_sender()

    # Enviar comando de seguridad al iniciar
    send_manual_command()
    print("=== Control Manual Iniciado ===")
    print("Formato de entrada: Ux,Uy,Ut,Patada,Cilindro")
    #    0.0,0.0,0.0,0,0
    try:
        while True:
            line = input(">> ")
            cmd = parse_input(line)
            if cmd is None:
                print("Comando inválido. Debe ser: Ux,Uy,Ut,Patada,Cilindro")
                continue
            Ux, Uy, Ut, Patada, Cilindro = cmd
            send_manual_command(Ux, Uy, Ut, Patada, Cilindro)
            print(f"Enviado: M,{Ux},{Uy},{Ut},{Patada},{Cilindro},N,0,0")

    except KeyboardInterrupt:
        print("\nCerrando control manual...")
        send_manual_command()  # Detener robot

if __name__ == "__main__":
    main()