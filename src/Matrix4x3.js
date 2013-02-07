/////////////////////////////////////////////////////////////////////////////
//
// 3D Math Primer for Games and Graphics Development
//
// Matrix4x3.h - Declarations for class Matrix4x3
//
// For more details, see Matrix4x3.cpp
//
/////////////////////////////////////////////////////////////////////////////

/**
 * class Matrix4x3
 *
 * Implement a 4x3 transformation matrix.  This class can represent
 * any 3D affine transformation.

 * @brief Implements a 4x3 transformation matrix in 3-space.
 *
 * Implements a 4x3 transformation matrix--more accurately, a 4x4
 * transformation matrix with an implied right column of [0,0,0,1]^T.
 * Can represent any 3D affine transformation.
/**
 *
 * @param {Number} m11 Specifies row one, column one
 * @param {Number} m12 Specifies row one, column two
 * @param {Number} m13 Specifies row one, column three
 * @param {Number} m21 Specifies row two, column one
 * @param {Number} m22 Specifies row two, column two
 * @param {Number} m23 Specifies row two, column three
 * @param {Number} m31 Specifies row three, column one
 * @param {Number} m32 Specifies row three, column two
 * @param {Number} m33 Specifies row three, column three
 * @param {Number} tx  Specifies row four, column one.
 * @param {Number} ty  Specifies row four, column two.
 * @param {Number} tz  Specifies row four, column three.
 * @constructor
 */
function Matrix4x3(m11, m12, m13, m21, m22, m23, m31, m32, m33, tx, ty, tz) {
  // The values of the matrix.  Basically the upper 3x3 portion
  // contains a linear transformation, and the last row is the
  // translation portion.  See the Matrix4x3.cpp for more
  // details.
  this.m11 = m11;
  this.m12 = m12;
  this.m13 = m13;
  this.m21 = m21;
  this.m22 = m22;
  this.m23 = m23;
  this.m31 = m31;
  this.m32 = m32;
  this.m33 = m33;
  this.tx = tx;
  this.ty = ty;
  this.tz = tz;
}


/////////////////////////////////////////////////////////////////////////////
//
// 3D Math Primer for Games and Graphics Development
//
// Matrix4x3.cpp - Implementation of class Matrix4x3
//
// For more details see section 11.5.
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Notes:
//
// See Chapter 11 for more information on class design decisions.
//
/**
 *
 * MATRIX ORGANIZATION
 *
 * The purpose of this class is so that a user might perform transformations
 * without fiddling with plus or minus signs or transposing the matrix
 * until the output "looks right."  But of course, the specifics of the
 * internal representation is important.  Not only for the implementation
 * in this file to be correct, but occasionally direct access to the
 * matrix variables is necessary, or beneficial for optimization.  Thus,
 * we document our matrix conventions here.
 *
 * We use row vectors, so multiplying by our matrix looks like this:
 *
 *               | m11 m12 m13 |
 *     [ x y z ] | m21 m22 m23 | = [ x' y' z' ]
 *               | m31 m32 m33 |
 *               | tx  ty  tz  |
 *
 * Strict adherance to linear algebra rules dictates that this
 * multiplication is actually undefined.  To circumvent this, we can
 * consider the input and output vectors as having an assumed fourth
 * coordinate of 1.  Also, since we cannot technically invert a 4x3 matrix
 * according to linear algebra rules, we will also assume a rightmost
 * column of [ 0 0 0 1 ].  This is shown below:
 *
 *                 | m11 m12 m13 0 |
 *     [ x y z 1 ] | m21 m22 m23 0 | = [ x' y' z' 1 ]
 *                 | m31 m32 m33 0 |
 *                 | tx  ty  tz  1 |
 *
 * In case you have forgotten your linear algebra rules for multiplying
 * matrices (which are described in section 7.1.6 and 7.1.7), see the
 * definition of operator* for the expanded computations.
 *
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Matrix4x3 class members
//
/////////////////////////////////////////////////////////////////////////////

/**
 * Matrix4x3.prototype.identity
 *
 * Set the matrix to identity
 */
Matrix4x3.prototype.identity = function () {
  this.m11 = 1;
  this.m12 = 0;
  this.m13 = 0;
  this.m21 = 0;
  this.m22 = 1;
  this.m23 = 0;
  this.m31 = 0;
  this.m32 = 0;
  this.m33 = 1;
  this.tx = this.ty = this.tz = 0;
}

/**
 * Matrix4x3.prototype.zeroTranslation
 *
 * Zero the 4th row of the matrix, which contains the translation portion.
 */
Matrix4x3.prototype.zeroTranslation = function () {
  this.tx = this.ty = this.tz = 0;
}

/**
 * Matrix4x3.prototype.setTranslation
 *
 * Sets the translation portion of the matrix in vector form

 * @param {Vector3} d Specifies the vector to be assigned.
 */
Matrix4x3.prototype.setTranslation = function (d) {
  this.tx = d.x;
  this.ty = d.y;
  this.tz = d.z;
}

/**
 * Matrix4x3.prototype.setTranslation
 *
 * Sets the translation portion of the matrix in vector form

 * @param {Vector3} d Specifies the vector to be assigned as translation.
 */
Matrix4x3.prototype.setupTranslation = function (d) {
  // Set the linear transformation portion to identity
  this.m11 = 1;
  this.m12 = 0;
  this.m13 = 0;
  this.m21 = 0;
  this.m22 = 1;
  this.m23 = 0;
  this.m31 = 0;
  this.m32 = 0;
  this.m33 = 1;

  // Set the translation portion
  this.tx = d.x;
  this.ty = d.y;
  this.tz = d.z;
}

/**
 * Matrix4x3.prototype.setupLocalToParent
 *
 * Setup the matrix to perform a local -> parent transformation, given
 * the position and orientation of the local reference frame within the
 * parent reference frame.
 *
 * A very common use of this will be to construct a object -> world matrix.
 * As an example, the transformation in this case is straightforward.  We
 * first rotate from object space into inertial space, then we translate
 * into world space.
 *
 * We allow the orientation to be specified using either euler angles,
 * or a RotationMatrix

 * @param {Vector3} pos Specifies the position of the local space within the
 *     parent space.
 * @param {EulerAngles} orient Specifies the orientation of the local space within
 *     the parent space as an Euler angle triplet.
 */
Matrix4x3.prototype.setupLocalToParent = function (pos, orient) {
  // Create a rotation matrix.
  var orientMatrix = new RotationMatrix()
    ;
  orientMatrix.setup(orient);

  // Setup the 4x3 matrix.  Note: if we were really concerned with
  // speed, we could create the matrix directly into these variables,
  // without using the temporary RotationMatrix object.  This would
  // save us a function call and a few copy operations.
  this.setupLocalToParent(pos, orientMatrix);
};

/**
 * @param {Vector3} pos Specifies the position of the local space within the
 *     parent space.
 * @param {RotationMatrix} orient Specifies the orientation of the local space within
 *     the parent space as a rotation matrix.
 */
Matrix4x3.prototype.setupLocalToParent = function (pos, orient) {
  // Copy the rotation portion of the matrix.  According to
  // the comments in RotationMatrix.cpp, the rotation matrix
  // is "normally" an inertial->object matrix, which is
  // parent->local.  We want a local->parent rotation, so we
  // must transpose while copying
  this.m11 = orient.m11;
  this.m12 = orient.m21;
  this.m13 = orient.m31;
  this.m21 = orient.m12;
  this.m22 = orient.m22;
  this.m23 = orient.m32;
  this.m31 = orient.m13;
  this.m32 = orient.m23;
  this.m33 = orient.m33;

  // Now set the translation portion.  Translation happens "after"
  // the 3x3 portion, so we can simply copy the position
  // field directly
  this.tx = pos.x;
  this.ty = pos.y;
  this.tz = pos.z;
}

/**
 * Matrix4x3.prototype.setupParentToLocal
 *
 * Setup the matrix to perform a parent -> local transformation, given
 * the position and orientation of the local reference frame within the
 * parent reference frame.
 *
 * A very common use of this will be to construct a world -> object matrix.
 * To perform this transformation, we would normally FIRST transform
 * from world to inertial space, and then rotate from inertial space into
 * object space.  However, out 4x3 matrix always translates last.  So
 * we think about creating two matrices T and R, and then concatenating
 * M = TR.
 *
 * We allow the orientation to be specified using either euler angles,
 * or a RotationMatrix

 * @param {Vector3} pos Specifies the position of the local space within the
 *     parent space.
 * @param {EulerAngles} orient Specifies the orientation of the local space within
 *     the parent space as an Euler angle triplet.
 */
Matrix4x3.prototype.setupParentToLocal = function (pos, orient) {
  // Create a rotation matrix.
  var orientMatrix = new RotationMatrix()
    ;
  orientMatrix.setup(orient);

  // Setup the 4x3 matrix.
  this.setupParentToLocal(pos, orientMatrix);
}

/**
 * @param {Vector3} pos Specifies the position of the local space within the
 *     parent space.
 * @param {RotationMatrix} orient Specifies the orientation of the local space within
 *     the parent space as a rotation matrix.
 */
Matrix4x3.prototype.setupParentToLocal = function (pos, orient) {
  // Copy the rotation portion of the matrix.  We can copy the
  // elements directly (without transposing) according
  // to the layout as commented in RotationMatrix.cpp
  this.m11 = orient.m11;
  this.m12 = orient.m12;
  this.m13 = orient.m13;
  this.m21 = orient.m21;
  this.m22 = orient.m22;
  this.m23 = orient.m23;
  this.m31 = orient.m31;
  this.m32 = orient.m32;
  this.m33 = orient.m33;

  // Now set the translation portion.  Normally, we would
  // translate by the negative of the position to translate
  // from world to inertial space.  However, we must correct
  // for the fact that the rotation occurs "first."  So we
  // must rotate the translation portion.  This is the same
  // as create a translation matrix T to translate by -pos,
  // and a rotation matrix R, and then creating the matrix
  // as the concatenation of TR
  this.tx = -(pos.x * this.m11 + pos.y * this.m21 + pos.z * this.m31);
  this.ty = -(pos.x * this.m12 + pos.y * this.m22 + pos.z * this.m32);
  this.tz = -(pos.x * this.m13 + pos.y * this.m23 + pos.z * this.m33);
}

/**
 * @param {Number} theta Specifies the angle of rotation.
 */
Matrix4x3.prototype.setupRotateX = function (theta) {
  // Get sin and cosine of rotation angle
  var s = sin(theta)
    , c = cos(theta)
    ;
  this.m11 = 1;
  this.m12 = 0;
  this.m13 = 0;
  this.m21 = 0;
  this.m22 = c;
  this.m23 = s;
  this.m31 = 0;
  this.m32 = -s;
  this.m33 = c;
  this.tx = this.ty = this.tz = 0;
}

/**
 * @param {Number} theta Specifies the angle of rotation.
 */
Matrix4x3.prototype.setupRotateY = function (theta) {
  // Get sin and cosine of rotation angle
  var s = sin(theta)
    , c = cos(theta)
    ;
  this.m11 = c;
  this.m12 = 0;
  this.m13 = -s;
  this.m21 = 0;
  this.m22 = 1;
  this.m23 = 0;
  this.m31 = s;
  this.m32 = 0;
  this.m33 = c;
  this.tx = this.ty = this.tz = 0;
}

/**
 * @param {Number} theta Specifies the angle of rotation.
 */
Matrix4x3.prototype.setupRotateZ = function (theta) {
  // Get sin and cosine of rotation angle
  var s = sin(theta)
    , c = cos(theta)
    ;
  this.m11 = c;
  this.m12 = s;
  this.m13 = 0;
  this.m21 = -s;
  this.m22 = c;
  this.m23 = 0;
  this.m31 = 0;
  this.m32 = 0;
  this.m33 = 1;
  this.tx = this.ty = this.tz = 0;
}

/**
 * Matrix4x3.prototype.setupRotate
 *
 * Setup the matrix to perform a rotation about an arbitrary axis.
 * The axis of rotation must pass through the origin.
 *
 * axis defines the axis of rotation, and must be a unit vector.
 *
 * theta is the amount of rotation, in radians.  The left-hand rule is
 * used to define "positive" rotation.
 *
 * The translation portion is reset.
 *
 * See 8.2.3 for more info.

 * @param {Vector3} axis Specifies the axis of rotation.
 * @param {Number} theta Specifies the angle of rotation.
 */
Matrix4x3.prototype.setupRotate = function (axis, theta) {
  // Quick sanity check to make sure they passed in a unit vector
  // to specify the axis
  assert(abs(axis * axis - 1) < 1);

  // Get sin and cosine of rotation angle
  var s = sin(theta)
    , c = cos(theta)
    ;

  // Compute 1 - cos(theta) and some common subexpressions
  var a = 1 - c
    , ax = a * axis.x
    , ay = a * axis.y
    , az = a * axis.z
    ;

  // Set the matrix elements.  There is still a little more
  // opportunity for optimization due to the many common
  // subexpressions.  We'll let the compiler handle that...

  this.m11 = ax * axis.x + c;
  this.m12 = ax * axis.y + axis.z * s;
  this.m13 = ax * axis.z - axis.y * s;

  this.m21 = ay * axis.x - axis.z * s;
  this.m22 = ay * axis.y + c;
  this.m23 = ay * axis.z + axis.x * s;

  this.m31 = az * axis.x + axis.y * s;
  this.m32 = az * axis.y - axis.x * s;
  this.m33 = az * axis.z + c;

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
};

/**
 * Matrix4x3.prototype.fromQuaternion
 *
 * Setup the matrix to perform a rotation, given the angular displacement
 * in quaternion form.
 *
 * The translation portion is reset.
 *
 * See 10.6.3 for more info.

 * @param {Quaternion} q Specifies the quaternion to be converted.
 */
Matrix4x3.prototype.fromQuaternion = function (q) {
  // Compute a few values to optimize common subexpressions
  var ww = 2 * q.w
    , xx = 2 * q.x
    , yy = 2 * q.y
    , zz = 2 * q.z
    ;

  // Set the matrix elements.  There is still a little more
  // opportunity for optimization due to the many common
  // subexpressions.  We'll let the compiler handle that...

  this.m11 = 1 - yy * q.y - zz * q.z;
  this.m12 = xx * q.y + ww * q.z;
  this.m13 = xx * q.z - ww * q.x;

  this.m21 = xx * q.y - ww * q.z;
  this.m22 = 1 - xx * q.x - zz * q.z;
  this.m23 = yy * q.z + ww * q.x;

  this.m31 = xx * q.z + ww * q.y;
  this.m32 = yy * q.z - ww * q.x;
  this.m33 = 1 - xx * q.x - yy * q.y;

  // Reset the translation portion

  this.tx = this.ty = this.tz = 0;
};

/**
 * Matrix4x3.prototype.setupScale
 *
 * Setup the matrix to perform scale on each axis.  For uniform scale by k,
 * use a vector of the form Vector3(k,k,k)
 *
 * The translation portion is reset.
 *
 * See 8.3.1 for more info.

 * @param {Vector3} s Specifies the scaling factor on each axis.
 */
Matrix4x3.prototype.setupScale = function (s) {
  // Set the matrix elements.  Pretty straightforward
  this.m11 = s.x;
  this.m12 = 0;
  this.m13 = 0;
  this.m21 = 0;
  this.m22 = s.y;
  this.m23 = 0;
  this.m31 = 0;
  this.m32 = 0;
  this.m33 = s.z;

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
};

/**
 * Matrix4x3.prototype.setupScaleAlongAxis
 *
 * Setup the matrix to perform scale along an arbitrary axis.
 *
 * The axis is specified using a unit vector.
 *
 * The translation portion is reset.
 *
 * See 8.3.2 for more info.

 * @param {Vector3} axis Specifies the axis of scaling.
 * @param {Number} k Specifies the scaling factor.
 */
Matrix4x3.prototype.setupScaleAlongAxis = function (axis, k) {
  // Quick sanity check to make sure they passed in a unit vector
  // to specify the axis
  assert(abs(axis * axis - 1) < 1);

  // Compute k-1 and some common subexpressions
  var a = k - 1
    , ax = a * axis.x
    , ay = a * axis.y
    , az = a * axis.z
    ;

  // Fill in the matrix elements.  We'll do the common
  // subexpression optimization ourselves here, since diagonally
  // opposite matrix elements are equal
  this.m11 = ax * axis.x + 1;
  this.m22 = ay * axis.y + 1;
  this.m32 = az * axis.z + 1;

  this.m12 = this.m21 = ax * axis.y;
  this.m13 = this.m31 = ax * axis.z;
  this.m23 = this.m32 = ay * axis.z;

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
}

/**
 * Matrix4x3.prototype.setupShear
 *
 * Setup the matrix to perform a shear
 *
 * The type of shear is specified by the 1-based "axis" index.  The effect
 * of transforming a point by the matrix is described by the pseudocode
 * below:
 *
 *	axis == 1  =>  y += s*x, z += t*x
 *	axis == 2  =>  x += s*y, z += t*y
 *	axis == 3  =>  x += s*z, y += t*z
 *
 * The translation portion is reset.
 *
 * See 8.6 for more info.

 * @param {int} axis Specifies the axis as follows:
 *   <ul>
 *     <li>1 -- Shears along the x-axis (y += s*x; z += t*x)</li>
 *     <li>2 -- Shears along the y-axis (x += s*y; z += t*y)</li>
 *     <li>3 -- Shears along the z-axis (x += s*z; y += t*z)</li>
 *   </ul>
 * @param {Number} s Specifies the first shearing coefficient.
 * @param {Number} t Specifies the second shearing coefficient.
 * @warning Asserts that axis is 1, 2, or 3.  Recommend splitting
 *     into three functions, as I see no benefit from specifying
 *     the axis numerically and using an assert.
 */
Matrix4x3.prototype.setupShear = function (axis, s, t) {
  // Check which type of shear they want
  switch (axis) {

    case 1: // Shear y and z using x
      this.m11 = 1;
      this.m12 = s;
      this.m13 = t;
      this.m21 = 0;
      this.m22 = 1;
      this.m23 = 0;
      this.m31 = 0;
      this.m32 = 0;
      this.m33 = 1;
      break;

    case 2: // Shear x and z using y
      this.m11 = 1;
      this.m12 = 0;
      this.m13 = 0;
      this.m21 = s;
      this.m22 = 1;
      this.m23 = t;
      this.m31 = 0;
      this.m32 = 0;
      this.m33 = 1;
      break;

    case 3: // Shear x and y using z
      this.m11 = 1;
      this.m12 = 0;
      this.m13 = 0;
      this.m21 = 0;
      this.m22 = 1;
      this.m23 = 0;
      this.m31 = s;
      this.m32 = t;
      this.m33 = 1;
      break;

    default:
      // bogus axis index
      assert(false);
  }

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
}

/**
 * Matrix4x3.prototype.setupProject
 *
 * Setup the matrix to perform a projection onto a plane passing
 * through the origin.  The plane is perpendicular to the
 * unit vector n.
 *
 * See 8.4.2 for more info.

 * @param {Vector3} n Specifies the normal to the plane.
 */
Matrix4x3.prototype.setupProject = function (n) {
  // Quick sanity check to make sure they passed in a unit vector
  // to specify the axis
  assert(abs(n * n - 1) < 1);

  // Fill in the matrix elements.  We'll do the common
  // subexpression optimization ourselves here, since diagonally
  // opposite matrix elements are equal
  this.m11 = 1 - n.x * n.x;
  this.m22 = 1 - n.y * n.y;
  this.m33 = 1 - n.z * n.z;

  this.m12 = this.m21 = -n.x * n.y;
  this.m13 = this.m31 = -n.x * n.z;
  this.m23 = this.m32 = -n.y * n.z;

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
};

/**
 * Matrix4x3.prototype.setupReflect
 *
 * Setup the matrix to perform a reflection about a plane parallel
 * to a cardinal plane.
 *
 * axis is a 1-based index which specifies the plane to project about:
 *
 *	1 => reflect about the plane x=k
 *	2 => reflect about the plane y=k
 *	3 => reflect about the plane z=k
 *
 * The translation is set appropriately, since translation must occur if
 * k != 0
 *
 * See 8.5 for more info.

 * @param {int} axis Specifies the axis as follows:
 *   <ul>
 *     <li>1 -- Reflect about the plane x=k
 *     <li>2 -- Reflect about the plane y=k
 *     <li>3 -- Reflect about the plane z=k
 *   </ul>
 * @param {Number} k Specifies the plane displacement along the axis.
 * @warning Asserts that axis is 1, 2, or 3.  Recommend splitting
 *     into three functions, as I see no benefit from specifying
 *     the axis numerically and using an assert.
 */
Matrix4x3.prototype.setupReflect = function (axis, k) {
  if (k == null) {
    k = 0;
  }

  // Check which plane they want to reflect about
  switch (axis) {

    case 1: // Reflect about the plane x=k
      this.m11 = -1;
      this.m12 = 0;
      this.m13 = 0;
      this.m21 = 0;
      this.m22 = 1;
      this.m23 = 0;
      this.m31 = 0;
      this.m32 = 0;
      this.m33 = 1;
      this.tx = 2 * k;
      this.ty = 0;
      this.tz = 0;
      break;

    case 2: // Reflect about the plane y=k
      this.m11 = 1;
      this.m12 = 0;
      this.m13 = 0;
      this.m21 = 0;
      this.m22 = -1;
      this.m23 = 0;
      this.m31 = 0;
      this.m32 = 0;
      this.m33 = 1;
      this.tx = 0;
      this.ty = 2 * k;
      this.tz = 0;
      break;

    case 3: // Reflect about the plane z=k
      this.m11 = 1;
      this.m12 = 0;
      this.m13 = 0;
      this.m21 = 0;
      this.m22 = 1;
      this.m23 = 0;
      this.m31 = 0;
      this.m32 = 0;
      this.m33 = -1;
      this.tx = 0;
      this.ty = 0;
      this.tz = 2 * k;
      break;

    default:
      // bogus axis index
      assert(false);
  }

};

/**
 * Matrix4x3.prototype.setupReflect
 *
 * Setup the matrix to perform a reflection about an arbitrary plane
 * through the origin.  The unit vector n is perpendicular to the plane.
 *
 * The translation portion is reset.
 *
 * See 8.5 for more info.

 * @param {Vector3} n Specifies the normal to the plane.
 */
Matrix4x3.prototype.setupReflect = function (n) {
  // Quick sanity check to make sure they passed in a unit vector
  // to specify the axis
  assert(abs(n * n - 1) < 1);

  // Compute common subexpressions
  var ax = -2 * n.x
    , ay = -2 * n.y
    , az = -2 * n.z
    ;

  // Fill in the matrix elements.  We'll do the common
  // subexpression optimization ourselves here, since diagonally
  // opposite matrix elements are equal
  this.m11 = 1 + ax * n.x;
  this.m22 = 1 + ay * n.y;
  this.m32 = 1 + az * n.z;
  this.m12 = this.m21 = ax * n.y;
  this.m13 = this.m31 = ax * n.z;
  this.m23 = this.m32 = ay * n.z;

  // Reset the translation portion
  this.tx = this.ty = this.tz = 0;
};

/**
 * Matrix4x3.prototype.setupReflect
 *
 * Setup the matrix to perform a reflection over a given plane
 * in quaternion form.

 * @param {Plane} plane Plane to reflect over.
 */
Matrix4x3.prototype.setupReflect = function (plane) {
  this.m11 = -2 * plane.a * plane.a + 1;
  this.m12 = -2 * plane.b * plane.a;
  this.m13 = -2 * plane.c * plane.a;

  this.m21 = -2 * plane.a * plane.b;
  this.m22 = -2 * plane.b * plane.b + 1;
  this.m23 = -2 * plane.c * plane.b;

  this.m31 = -2 * plane.a * plane.c;
  this.m32 = -2 * plane.b * plane.c;
  this.m33 = -2 * plane.c * plane.c + 1;

  this.tx = -2 * plane.a * plane.d;
  this.ty = -2 * plane.b * plane.d;
  this.tz = -2 * plane.c * plane.d;
};

/**
 * determinant
 *
 * Compute the determinant of the 3x3 portion of the matrix.
 *
 * See 9.1.1 for more info.

 * @return {Number} The determinant of the matrix.
 */
Matrix4x3.prototype.determinant = function () {
  return this.m11 * (this.m22 * this.m33 - this.m23 * this.m32)
    + this.m12 * (this.m23 * this.m31 - this.m21 * this.m33)
    + this.m13 * (this.m21 * this.m32 - this.m22 * this.m31);
};

/**
 * inverse
 *
 * Compute the inverse of a matrix.  We use the classical adjoint divided
 * by the determinant method.
 *
 * See 9.2.1 for more info.

 * @return {Matrix4x3} The inverse of the matrix.
 */
Matrix4x3.prototype.inverse = function () {
  // Compute the determinant
  var det = this.determinant()
    ;

// If we're singular, then the determinant is zero and there's
// no inverse

  assert(abs(det) > 000001);

// Compute one over the determinant, so we divide once and
// can *multiply* per element

  var oneOverDet = 1 / det
    ;

// Compute the 3x3 portion of the inverse, by
// dividing the adjoint by the determinant
  var r = new Matrix4x3()
    ;

  r.m11 = (this.m22 * this.m33 - this.m23 * this.m32) * oneOverDet;
  r.m12 = (this.m13 * this.m32 - this.m12 * this.m33) * oneOverDet;
  r.m13 = (this.m12 * this.m23 - this.m13 * this.m22) * oneOverDet;

  r.m21 = (this.m23 * this.m31 - this.m21 * this.m33) * oneOverDet;
  r.m22 = (this.m11 * this.m33 - this.m13 * this.m31) * oneOverDet;
  r.m23 = (this.m13 * this.m21 - this.m11 * this.m23) * oneOverDet;

  r.m31 = (this.m21 * this.m32 - this.m22 * this.m31) * oneOverDet;
  r.m32 = (this.m12 * this.m31 - this.m11 * this.m32) * oneOverDet;
  r.m33 = (this.m11 * this.m22 - this.m12 * this.m21) * oneOverDet;

// Compute the translation portion of the inverse

  r.tx = -(this.tx * r.m11 + this.ty * r.m21 + this.tz * r.m31);
  r.ty = -(this.tx * r.m12 + this.ty * r.m22 + this.tz * r.m32);
  r.tz = -(this.tx * r.m13 + this.ty * r.m23 + this.tz * r.m33);

// Return it.  Ouch - involves a copy constructor call.  If speed
// is critical, we may need a separate function which places the
// result where we want it...

  return r;
};

/**
 * getTranslation
 *
 * Return the translation row of the matrix in vector form

 * @return {Vector3} The translation vector of the matrix.
 */
Matrix4x3.prototype.getTranslation = function () {
  return new Vector3(this.tx, this.ty, this.tz);
}

/**
 * getPositionFromParentToLocalMatrix
 *
 * Extract the position of an object given a parent -> local transformation
 * matrix (such as a world -> object matrix)
 *
 * We assume that the matrix represents a rigid transformation.  (No scale,
 * skew, or mirroring)

 * @return {Vector3} The position portion of the matrix.
 */
Matrix4x3.prototype.getPositionFromParentToLocalMatrix = function () {
  // Multiply negative translation value by the
  // transpose of the 3x3 portion.  By using the transpose,
  // we assume that the matrix is orthogonal.  (This function
  // doesn't really make sense for non-rigid transformations...)
  return new Vector3(
    -(this.tx * this.m11 + this.ty * this.m12 + this.tz * this.m13),
    -(this.tx * this.m21 + this.ty * this.m22 + this.tz * this.m23),
    -(this.tx * this.m31 + this.ty * this.m32 + this.tz * this.m33)
  );
};

/**
 * getPositionFromLocalToParentMatrix
 *
 * Extract the position of an object given a local -> parent transformation
 * matrix (such as an object -> world matrix)

 * @return {Vector3} The position portion of the matrix.
 */
Matrix4x3.prototype.getPositionFromLocalToParentMatrix = function () {
  // Position is simply the translation portion
  return new Vector3(this.tx, this.ty, this.tz);
}

/**
 * Vector * Matrix4x3
 *
 * Transform the point.  This makes using the vector class look like it
 * does with linear algebra notation on paper.
 *
 * See 7.1.7
 *
 * @param {Vector3} p Specifies the vector multiplicand.
 * @param {Matrix4x3} m Specifies the matrix multiplicator.
 * @return {Vector3} The vector transformed by the matrix.
 */
Matrix4x3.multiply = function (p, m) {
  // Grind through the linear algebra.
  return new Vector3(
    p.x * m.m11 + p.y * m.m21 + p.z * m.m31 + m.tx,
    p.x * m.m12 + p.y * m.m22 + p.z * m.m32 + m.ty,
    p.x * m.m13 + p.y * m.m23 + p.z * m.m33 + m.tz
  );
};

/**
 * Matrix4x3 * Matrix4x3
 *
 * Matrix concatenation.  This makes using the vector class look like it
 * does with linear algebra notation on paper.
 *
 * We also provide a *= operator, as per C convention.
 *
 * See 7.1.6
 *
 * @param {Matrix4x3} a Specifies the first matrix.
 * @param {Matrix4x3} b Specifies the second matrix.
 * @return {Matrix4x3} The product of the two matrices.
 */
Matrix4x3.concat = function (a, b) {
  var r = new Matrix4x3()
    ;

  // Compute the upper 3x3 (linear transformation) portion

  r.m11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
  r.m12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
  r.m13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;

  r.m21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
  r.m22 = a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
  r.m23 = a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;

  r.m31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
  r.m32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
  r.m33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;

  // Compute the translation portion
  r.tx = a.tx * b.m11 + a.ty * b.m21 + a.tz * b.m31 + b.tx;
  r.ty = a.tx * b.m12 + a.ty * b.m22 + a.tz * b.m32 + b.ty;
  r.tz = a.tx * b.m13 + a.ty * b.m23 + a.tz * b.m33 + b.tz;

  // Return it.  Ouch - involves a copy constructor call.  If speed
  // is critical, we may need a separate function which places the
  // result where we want it...
  return r;
};