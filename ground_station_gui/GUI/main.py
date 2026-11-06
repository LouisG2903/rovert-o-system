import interface

if __name__ == "__main__":
    # instancia de aplicacion
    app = interface.App(title="Interface", size=(1920, 1080))
    app.rover_params.set_ambiente()
    app.rover_params.set_bateria(50)
    app.mainloop()
