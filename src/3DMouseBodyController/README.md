# Fusion360 Controller

## Fusion360 AddIn
The directory Fusion360ControllerAddIn is a Fusion360 AddIn and it can be added as an AddIn in Fusion360 directly.

## Client
The client is a python script that sends commands in json format to the AddIn. There are two commands: rotate and translate.

### Rotate
```JSON
{
    "command": "rotate",
    "rx": rx,
    "ry": ry,
    "rz": rz
}
```
where
* rx: float, rotate camera on x-asix by _rx_ degree.
* ry: float, rotate camera on y-asix by _ry_ degree.
* rz: float, rotate camera on z-asix by _rz_ degree.


### Translate
```JSON
{
    "command": "translate",
    "tx": tx,
    "ty": ty,
    "tz": tz
}
```
where
* tx: float, move camera along x-asix by _tx_ unit.
* ty: float, move camera along y-asix by _ty_ unit.
* tz: float, move camera along z-asix by _tz_ unit.

