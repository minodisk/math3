/**
 * class RotationMatrix
 *
 * Implement a simple 3x3 matrix that is used for ROTATION ONLY.  The
 * matrix is assumed to be orthogonal.  The direction of transformation
 * is specified at the time of transformation.
 *
 * For more details see section 11.4.
 */

/**
 * Implements a 3x3 rotation matrix in 3-space.
 * The matrix is assumed to be orthogonal.
 * @param {Number} m11 Specifies row one, column one
 * @param {Number} m12 Specifies row one, column two
 * @param {Number} m13 Specifies row one, column three
 * @param {Number} m21 Specifies row two, column one
 * @param {Number} m22 Specifies row two, column two
 * @param {Number} m23 Specifies row two, column three
 * @param {Number} m31 Specifies row three, column one
 * @param {Number} m32 Specifies row three, column two
 * @param {Number} m33 Specifies row three, column three
 * @constructor
 */
function RotationMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33) {
  // The 9 values of the matrix.  See RotationMatrix.cpp file for
  // the details of the layout
  this.m11 = m11;
  this.m12 = m12;
  this.m13 = m13;
  this.m21 = m21;
  this.m22 = m22;
  this.m23 = m23;
  this.m31 = m31;
  this.m32 = m32;
  this.m33 = m33;
}

/////////////////////////////////////////////////////////////////////////////
//
// class RotationMatrix
//
//---------------------------------------------------------------------------
//
// MATRIX ORGANIZATION
//
// A user of this class should rarely care how the matrix is organized.
// However, it is of course important that internally we keep everything
// straight.
//
// The matrix is assumed to be a rotation matrix only, and therefore
// orthogonal.  The "forward" direction of transformation (if that really
// even applies in this case) will be from inertial to object space.
// To perform an object->inertial rotation, we will multiply by the
// transpose.
//
// In other words:
//
// Inertial to object:
//
//                  | m11 m12 m13 |
//     [ ix iy iz ] | m21 m22 m23 | = [ ox oy oz ]
//                  | m31 m32 m33 |
//
// Object to inertial:
//
//                  | m11 m21 m31 |
//     [ ox oy oz ] | m12 m22 m32 | = [ ix iy iz ]
//                  | m13 m23 m33 |
//
// Or, using column vector notation:
//
// Inertial to object:
//
//     | m11 m21 m31 | | ix |   | ox |
//     | m12 m22 m32 | | iy | = | oy |
//     | m13 m23 m33 | | iz |   | oz |
//
// Object to inertial:
//
//     | m11 m12 m13 | | ox |   | ix |
//     | m21 m22 m23 | | oy | = | iy |
//     | m31 m32 m33 | | oz |   | iz |
//
/////////////////////////////////////////////////////////////////////////////

/**
 * Set the matrix to the identity matrix
 */
RotationMatrix.prototype.identity = function () {
  this.m11 = 1;
  this.m12 = 0;
  this.m13 = 0;

  this.m21 = 0;
  this.m22 = 1;
  this.m23 = 0;

  this.m31 = 0;
  this.m32 = 0;
  this.m33 = 1;
};

/**
 * Setup the matrix with the specified orientation
 *
 * See 10.6.1
 *
 * @param {EulerAngles} orientation Specifies the Euler triplet to be converted.
 */
RotationMatrix.prototype.setup = function (orientation) {
  // Fetch sine and cosine of angles
  var sh = sin(orientation.heading)
    , sp = sin(orientation.pitch)
    , sb = sin(orientation.bank)
    , ch = cos(orientation.heading)
    , cp = cos(orientation.pitch)
    , cb = cos(orientation.bank)
    ;

  // Fill in the matrix elements

  this.m11 = ch * cb + sh * sp * sb;
  this.m12 = -ch * sb + sh * sp * cb;
  this.m13 = sh * cp;

  this.m21 = sb * cp;
  this.m22 = cb * cp;
  this.m23 = -sp;

  this.m31 = -sh * cb + ch * sp * sb;
  this.m32 = sb * sh + ch * sp * cb;
  this.m33 = ch * cp;
};

/**
 * Setup the matrix, given a quaternion that performs an inertial->object
 * rotation
 *
 * See 10.6.3
 *
 * Initializes the matrix from a quaternion, assuming the quaternion
 * performs the rotation from inertial to object space.
 * @param {Quaternion} q Specifies the quaternion to be converted.
 */
RotationMatrix.prototype.fromInertialToObjectQuaternion = function (q) {
  // Fill in the matrix elements.  This could possibly be
  // optimized since there are many common subexpressions.
  // We'll leave that up to the compiler...

  this.m11 = 1 - 2 * (q.y * q.y + q.z * q.z);
  this.m12 = 2 * (q.x * q.y + q.w * q.z);
  this.m13 = 2 * (q.x * q.z - q.w * q.y);

  this.m21 = 2 * (q.x * q.y - q.w * q.z);
  this.m22 = 1 - 2 * (q.x * q.x + q.z * q.z);
  this.m23 = 2 * (q.y * q.z + q.w * q.x);

  this.m31 = 2 * (q.x * q.z + q.w * q.y);
  this.m32 = 2 * (q.y * q.z - q.w * q.x);
  this.m33 = 1 - 2 * (q.x * q.x + q.y * q.y);

};

/**
 * Setup the matrix, given a quaternion that performs an object->inertial
 * rotation
 *
 * See 10.6.3
 *
 * Initializes the matrix from a quaternion, assuming the quaternion
 * performs the rotation from object to inertial space.
 * @param {Quaternion} q Specifies the quaternion to be converted.
 */
RotationMatrix.prototype.fromObjectToInertialQuaternion = function (q) {
  // Fill in the matrix elements.  This could possibly be
  // optimized since there are many common subexpressions.
  // We'll leave that up to the compiler...

  this.m11 = 1 - 2 * (q.y * q.y + q.z * q.z);
  this.m12 = 2 * (q.x * q.y - q.w * q.z);
  this.m13 = 2 * (q.x * q.z + q.w * q.y);

  this.m21 = 2 * (q.x * q.y + q.w * q.z);
  this.m22 = 1 - 2 * (q.x * q.x + q.z * q.z);
  this.m23 = 2 * (q.y * q.z - q.w * q.x);

  this.m31 = 2 * (q.x * q.z - q.w * q.y);
  this.m32 = 2 * (q.y * q.z + q.w * q.x);
  this.m33 = 1 - 2 * (q.x * q.x + q.y * q.y);
};

/**
 * Rotate a vector from inertial to object space
 *
 * @param {Vector3} v Specifies the vector to be transformed.
 * @return {Vector3} The transformed vector.
 */
RotationMatrix.prototype.inertialToObject = function (v) {
  // Perform the matrix multiplication in the "standard" way.
  return new Vector3(
    this.m11 * v.x + this.m21 * v.y + this.m31 * v.z,
    this.m12 * v.x + this.m22 * v.y + this.m32 * v.z,
    this.m13 * v.x + this.m23 * v.y + this.m33 * v.z
  );
};

/**
 * Rotate a vector from object to inertial space
 *
 * @param {Vector3} v Specifies the vector to be transformed.
 * @return {Vector3} The transformed vector.
 */
RotationMatrix.prototype.objectToInertial = function (v) {
  // Multiply by the transpose
  return new Vector3(
    this.m11 * v.x + this.m12 * v.y + this.m13 * v.z,
    this.m21 * v.x + this.m22 * v.y + this.m23 * v.z,
    this.m31 * v.x + this.m32 * v.y + this.m33 * v.z
  );
};
