import numpy
from transformations import translation_matrix, quaternion_matrix, rotation_matrix, concatenate_matrices, inverse_matrix, quaternion_real, quaternion_imag, rotation_from_matrix, quaternion_from_matrix, translation_from_matrix, vector_norm


def to_homogeneous(vec):
	d = [float(x) for x in vec]

	if len(d) != 7 and len(d) != 3:
		raise ValueError("Only poses with 3 or 7 elements supported! this one has %d" % len(d))

	if len(d) == 7:
		return concatenate_matrices( translation_matrix(d[0:3]), quaternion_matrix([d[6]]+d[3:6]) )
	
	return concatenate_matrices( translation_matrix(d[0:2]+[0]), rotation_matrix(d[2], [0,0,1]) )

def from_homogeneous(M,n):
	if n != 7 and n != 3:
		raise ValueError("Only poses with 3 or 7 elements supported! this one has %d" % n)

	trans = translation_from_matrix(M)
	if n == 7:
		quat = quaternion_from_matrix(M)
		out = list(trans)
		out.extend(quaternion_imag(quat))
		out.append(quaternion_real(quat))
	else:
		ang, foo, bar = rotation_from_matrix(M)
		out = list(trans[0:2])
		out.append(ang)

	if len(out) != n:
		raise ValueError("Wrong output length: %d should be %d" %(len(out), n))

	return out

def compound(reference, diff, inv_diff = False):
	if len(reference) != len(diff):
		raise ValueError("reference and diff should be of same length! len(reference): %d, len(diff): %d" % (len(reference), len(diff)) )

	F_ref = to_homogeneous(reference)
	T_d =   to_homogeneous(diff)

	if inv_diff:
		T_d = inverse_matrix(T_d)

	F_d = concatenate_matrices( F_ref, T_d )

	return from_homogeneous(F_d, len(reference))


def relative_to_reference(reference, target):
	if len(reference) != len(target):
		raise ValueError("reference and target should be of same length! len(reference): %d, len(target): %d" % (len(reference), len(target)) )

	F_ref = to_homogeneous(reference)
	F_tar = to_homogeneous(target)

	F_ref_inv = inverse_matrix(F_ref)

	T_tar_ref = concatenate_matrices(F_ref_inv, F_tar)

	return from_homogeneous(T_tar_ref, len(reference))

def errors(reference, target):
	if len(reference) != len(target):
		raise ValueError("reference and target should be of same length! len(reference): %d, len(target): %d" % (len(reference), len(target)) )

	F_ref = to_homogeneous(reference)
	F_tar = to_homogeneous(target)

	F_ref_inv = inverse_matrix(F_ref)

	T_tar_ref = concatenate_matrices(F_ref_inv, F_tar)

	trans = translation_from_matrix(T_tar_ref)
	ang, foo, bar = rotation_from_matrix(T_tar_ref)

	return (numpy.sum( numpy.square(trans) ), ang*ang)
