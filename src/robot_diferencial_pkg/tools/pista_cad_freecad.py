"""
Modelo CAD de la pista propia en FreeCAD (requisito 5: pista diseñada en CAD, STL en metros).

    freecadcmd tools/pista_cad_freecad.py          (desde la raíz del paquete)

Flujo de diseño:
  1. tools/generar_pista.py define el trazado (línea central de rectas y arcos, ancho de
     carril y estrechamiento) y escribe los contornos de los muros en
     models/pista_propia/meshes/pista_propia.dxf.
  2. Este script arma el modelo en FreeCAD, en milímetros como cualquier CAD:
     contornos del DXF -> planta de cada muro (Part::Face, cara con hueco) ->
     extrusión de 300 mm (Part::Extrusion, sólido) -> pista (Part::Compound).
     Guarda models/pista_propia/cad/pista_propia.FCStd (se abre en FreeCAD con su árbol).
  3. Exporta la pista como STL binario EN METROS (escala 0.001) a
     models/pista_propia/meshes/pista_propia.stl, que es lo que carga Gazebo.
Para cambiar la altura de los muros: abrir el .FCStd, cambiar LengthFwd de las extrusiones,
recalcular y volver a exportar (o cambiar ALTO_MM aquí y correr el script).
"""
import os

import FreeCAD as App
import MeshPart
import Part

ALTO_MM = 300.0           # altura de muro (el enunciado pide >= 250 mm)


def raiz_paquete():
    """Raíz del paquete: el script se corre desde ella (o se pasa en PISTA_PKG)."""
    if os.environ.get('PISTA_PKG'):
        return os.environ['PISTA_PKG']
    if os.path.isdir(os.path.join(os.getcwd(), 'models', 'pista_propia')):
        return os.getcwd()
    raise SystemExit('Corra el script desde la raíz del paquete robot_diferencial_pkg')


def leer_dxf(ruta):
    """Polilíneas cerradas (POLYLINE/VERTEX) del DXF de generar_pista.py, en metros."""
    with open(ruta) as f:
        t = [s.strip() for s in f.read().split('\n')]
    polis, act, i = [], None, 0
    while i < len(t) - 1:
        cod, val = t[i], t[i + 1]
        if cod == '0' and val == 'POLYLINE':
            act = []
            polis.append(act)
        elif cod == '0' and val == 'VERTEX':
            x = y = None
            j = i + 2
            while j < len(t) - 1 and t[j] != '0':
                if t[j] == '10':
                    x = float(t[j + 1])
                elif t[j] == '20':
                    y = float(t[j + 1])
                j += 2
            act.append((x, y))
            i = j
            continue
        i += 2
    return polis


def contorno(doc, nombre, puntos_m):
    """Polígono cerrado en el plano z = 0, en mm."""
    v = [App.Vector(1000.0 * x, 1000.0 * y, 0.0) for x, y in puntos_m]
    obj = doc.addObject('Part::Feature', nombre)
    obj.Shape = Part.makePolygon(v + [v[0]])
    return obj


def muro(doc, nombre, borde_a, borde_b):
    """Crea la planta (cara entre dos contornos) y la extruye ALTO_MM hacia arriba."""
    planta = doc.addObject('Part::Face', 'Planta_' + nombre)
    planta.Sources = [borde_a, borde_b]
    ext = doc.addObject('Part::Extrusion', nombre)
    ext.Base = planta
    ext.DirMode = 'Custom'
    ext.Dir = App.Vector(0, 0, 1)
    ext.LengthFwd = ALTO_MM
    ext.Solid = True
    return ext


def main():
    raiz = raiz_paquete()
    d_modelo = os.path.join(raiz, 'models', 'pista_propia')
    polis = leer_dxf(os.path.join(d_modelo, 'meshes', 'pista_propia.dxf'))
    if len(polis) != 4:
        raise SystemExit(f'El DXF debe traer 4 contornos (2 por muro), trae {len(polis)}')

    doc = App.newDocument('pista_propia')
    nombres = ['Interior_borde_pista', 'Interior_borde_externo',
               'Exterior_borde_pista', 'Exterior_borde_externo']
    c = [contorno(doc, n, p) for n, p in zip(nombres, polis)]
    m_int = muro(doc, 'Muro_interior', c[0], c[1])
    m_ext = muro(doc, 'Muro_exterior', c[2], c[3])
    pista = doc.addObject('Part::Compound', 'Pista_propia')
    pista.Links = [m_int, m_ext]
    doc.recompute()
    # Sin interfaz gráfica no se guarda la vista: se deja visible solo el resultado final
    for obj in doc.Objects:
        obj.Visibility = obj is pista
    if not pista.Shape.isValid() or len(pista.Shape.Solids) != 2:
        raise SystemExit('El modelo no quedó con 2 sólidos válidos')

    d_cad = os.path.join(d_modelo, 'cad')
    os.makedirs(d_cad, exist_ok=True)
    doc.saveAs(os.path.join(d_cad, 'pista_propia.FCStd'))

    malla = MeshPart.meshFromShape(Shape=pista.Shape, LinearDeflection=0.5, AngularDeflection=0.2)
    escala = App.Matrix()
    escala.scale(0.001, 0.001, 0.001)           # mm (CAD) -> m (Gazebo)
    malla.transform(escala)
    ruta_stl = os.path.join(d_modelo, 'meshes', 'pista_propia.stl')
    malla.write(ruta_stl)
    caja = malla.BoundBox
    print(f'FCStd: {d_cad}/pista_propia.FCStd')
    print(f'STL  : {ruta_stl}  ({malla.CountFacets} triángulos, en metros)')
    print(f'caja : x[{caja.XMin:.2f}, {caja.XMax:.2f}] y[{caja.YMin:.2f}, {caja.YMax:.2f}] '
          f'z[{caja.ZMin:.2f}, {caja.ZMax:.2f}] m   '
          f'volumen sólido {pista.Shape.Volume / 1e9:.4f} m3')


main()
