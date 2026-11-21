"""Paquet de perception du projet Visual Follow-Me.

Limite les bibliotheques de calcul de numpy a un seul fil d'execution. Sur le CPU 2 coeurs du
projet, OpenVINO occupe deja tous les coeurs : les fils supplementaires d'OpenBLAS (utilises par
le tracker) se disputaient le processeur et divisaient le debit par deux (mesure du 2026-11-21).
Ces variables doivent etre definies avant le premier import de numpy, d'ou leur place ici :
Python execute ce fichier avant tout module du paquet.
"""
import os

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')