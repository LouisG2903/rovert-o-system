import interface

if __name__ == "__main__":
    # instancia de aplicacion
    app = interface.App(title="Interface", size=(1920, 1080))
    # PARA DIEGO que realizara la simulacion e inyeccion de datos a la interfaz
    # Utiliza las siguientes funciones y sustituye "a" por el valor que quieras asignar a la variable de la interfaz, ya sea int, float o str.
    # === app.rover_params.set_ambiente(a) ===
    # === app.rover_params.set_bateria(a) ===
    # === app.map_params.set_velocidad(a) ===
    # === app.map_params.set_aceleracion_x(a) ===
    # === app.map_params.set_aceleracion_y(a) ===
    # === app.map_params.set_aceleracion_z(a) ===
    app.mainloop()
