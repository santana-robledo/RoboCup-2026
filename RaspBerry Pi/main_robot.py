import main_delantero
import main_portero 

Portero = True
Delantero = False

if Portero == True and Delantero == False:
    main_portero.main()
    print("Portero")
elif Delantero == True and Portero == False:
    main_delantero.main()
    print("Delantero")
else:
    print("Seleccione un modelo válido")