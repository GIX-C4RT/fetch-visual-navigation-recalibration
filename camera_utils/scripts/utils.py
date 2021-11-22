from tf.transformations import *
def get_transform_matrix(transformstamped):
    tm = translation_matrix([transformstamped.transform.translation.x, transformstamped.transform.translation.y, transformstamped.transform.translation.z])
    qm = quaternion_matrix([transformstamped.transform.rotation.x, transformstamped.transform.rotation.y, transformstamped.transform.rotation.z, transformstamped.transform.rotation.w])
    return concatenate_matrices(tm, qm)