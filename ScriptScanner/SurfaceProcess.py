# Importamos libreria de mallado

import pymeshlab
import numpy as np

""" Clase para Mallado de Nube de Puntos """


class SurfaceGenerator:
    def __init__(self, scanner):
        # --------------------------------
        # Objeto scanner de interfaz

        self.scanner = scanner

        # --------------------------------
        # Crea el objeto Mesh

        self.k = 10
        self.smoothiter = 0

        # --------------------------------
        # Crea el objeto Mesh

        self.depth = 8
        self.fulldepth = 5
        self.cgdepth = 0
        self.scale = 1.1
        self.samplespernode = 1.5
        self.pointweight = 4
        self.iters = 8
        self.confidence = False
        self.preclean = False

        self.ms = None

    def generate_surface(self):
        # --------------------------------
        # Si el nombre de la carpeta del archivo ply no está vacío, ejecuta
        if self.scanner.ply_file_folder is not None:
            try:
                if self.scanner.inpt_normals_k.text() != "":
                    self.k = int(self.scanner.inpt_normals_k.text())
                else:
                    self.k = 10

                if self.scanner.inpt_normals_smoothiter.text() != "":
                    self.smoothiter = int(self.scanner.inpt_normals_smoothiter.text())
                else:
                    self.smoothiter = 0

                if self.scanner.inpt_reconstruction_depth.text() != "":
                    self.depth = int(self.scanner.inpt_reconstruction_depth.text())
                else:
                    self.depth = 8

                if self.scanner.inpt_octree_depth.text() != "":
                    self.fulldepth = int(self.scanner.inpt_octree_depth.text())
                else:
                    self.fulldepth = 5

                if self.scanner.inpt_gradients_depth.text() != "":
                    self.cgdepth = int(self.scanner.inpt_gradients_depth.text())
                else:
                    self.cgdepth = 0

                if self.scanner.inpt_scale.text() != "":
                    self.scale = float(self.scanner.inpt_scale.text())
                else:
                    self.scale = 1.1

                if self.scanner.inpt_nsamples.text() != "":
                    self.samplespernode = float(self.scanner.inpt_nsamples.text())
                else:
                    self.samplespernode = 1.5

                if self.scanner.inpt_interp_weight.text() != "":
                    self.scale = float(self.scanner.inpt_interp_weight.text())
                else:
                    self.scale = 4.0

                if self.scanner.inpt_gauss_seidel_relaxations.text() != "":
                    self.iters = int(self.scanner.inpt_gauss_seidel_relaxations.text())
                else:
                    self.iters = 8

                # --------------------------------
                # Crea el objeto Mesh
                self.ms = pymeshlab.MeshSet()

                # --------------------------------
                # Carga la nube de puntos dentro del objeto Mesh
                self.ms.load_new_mesh(self.scanner.ply_file_folder)

                # --------------------------------
                # Computa las normales de la nube de puntos

                self.ms.compute_normal_for_point_clouds(
                    k=int(self.k), smoothiter=int(self.smoothiter)
                )

                # --------------------------------
                # Genera la superficie de Poisson

                self.ms.generate_surface_reconstruction_screened_poisson(
                    depth=int(self.depth),
                    fulldepth=int(self.fulldepth),
                    cgdepth=int(self.cgdepth),
                    scale=float(self.scale),
                    samplespernode=float(self.samplespernode),
                    pointweight=float(self.pointweight),
                    iters=int(self.iters),
                    confidence=self.confidence,
                    preclean=self.preclean,
                )

            except Exception as e:
                print("Error: Mallado de superficie fallido -> " + str(e))
        else:
            print("Carpeta de nube de puntos no declarado.")

    def save_surface(self):
        try:
            # --------------------------------
            # Guarda el archivo Mesh

            if self.ms is not None:
                self.ms.save_current_mesh(
                    "EscaneosMallados3D/" + str(self.scanner.file_name) + ".ply",
                    save_face_color=True,
                )
                print("Mallado guardado -> " + str(self.scanner.file_name))
            else:
                print("Malla no generada, no ha sido posible guardar")
        except Exception as e:
            print("Error: No se pudo guardar mallado -> " + str(e))
