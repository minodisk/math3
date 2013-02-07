/////////////////////////////////////////////////////////////////////////////
//
// 3D Math Primer for Games and Graphics Development
//
// Quaternion.h - Declarations for class Quaternion
//
// For more details, see Quaternion.cpp
//
/////////////////////////////////////////////////////////////////////////////

/**
 * A quaternion-based representation of an orientation in 3-space.
 * @param {Number} [w=1] Specifies the w-coordinate.
 * @param {Number} [x=0] Specifies the x-coordinate.
 * @param {Number} [y=0] Specifies the y-coordinate.
 * @param {Number} [z=0] Specifies the z-coordinate.
 * @constructor
 */
function Quaternion(w, x, y, z) {
  // The 4 values of the quaternion.  Normally, it will not
  // be necessary to manipulate these directly.  However,
  // we leave them public, since prohibiting direct access
  // makes some operations, such as file I/O, unnecessarily
  // complicated.
  this.w = w != null ? w : 1;
  this.x = x != null ? x : 0;
  this.y = y != null ? y : 0;
  this.z = z != null ? z : 0;
}

/**
 * Set to identity
 * Sets this to the identity quaternion.
 */
Quaternion.prototype.identity = function () {
  this.w = 1;
  this.x = this.y = this.z = 0;
};


/**
 * Computes the dot product of two quaternions.
 * @param {Quaternion} a Specifies the first factor.
 * @param {Quaternion} b Specifies the second factor.
 * @return {Number} The dot product of the two quaternions.
 */
Quaternion.dotProduct = function (a, b) {
  return a.dotProduct(b);
};

/**
 * Computes the cross product of two quaternions.
 * @param {Quaternion} a Specifies the first factor.
 * @param {Quaternion} b Specifies the second factor.
 * @return {Quaternion} The cross product of the two quaternions.
 */
Quaternion.crossProduct = function (a, b) {
  return a.crossProduct(b);
};

/**
 * @param {Quaternion} q
 * @return {Quaternion}
 */
Quaternion.conjugate = function (q) {
  return q.conjugate();
};

/**
 * Performs quaternion exponentiation.
 * @param {Quaternion} q Specifies the base quaternion to be raised.
 * @param {Number} exponent Specifies the exponent.
 * @return {Quaternion} The quaternion \p q raised to the exponent \p exponent.
 */
Quaternion.pow = function (q, exponent) {
  return q.pow(exponent);
};

////////////////////////////////////////////////////////////////////////////
//
// 3D Math Primer for Games and Graphics Development
//
// Quaternion.cpp - Quaternion implementation
//
// For more details see section 11.3.
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Static variables
//
/////////////////////////////////////////////////////////////////////////////

/**
 * The identity quaternion.  Notice that there are no constructors
 * to the Quaternion class, since we really don't need any.
 * @type {Quaternion}
 */
Quaternion.kQuaternionIdentity = new Quaternion(1, 0, 0, 0);

/////////////////////////////////////////////////////////////////////////////
//
// class Quaternion members
//
/////////////////////////////////////////////////////////////////////////////

/**
 * Setup the quaternion to rotate about the specified axis
 * @param {Number} theta Specifies the angle of rotation.
 */
Quaternion.prototype.setToRotateAboutX = function (theta) {
  // Compute the half angle
  var thetaOver2 = theta * .5
    ;

  // Set the values
  this.w = cos(thetaOver2);
  this.x = sin(thetaOver2);
  this.y = 0;
  this.z = 0;
};

/**
 * Setup the quaternion to rotate about the specified axis
 * @param {Number} theta Specifies the angle of rotation.
 */
Quaternion.prototype.setToRotateAboutY = function (theta) {
  // Compute the half angle
  var thetaOver2 = theta * .5
    ;

  // Set the values
  this.w = cos(thetaOver2);
  this.x = 0;
  this.y = sin(thetaOver2);
  this.z = 0;
};

/**
 * Setup the quaternion to rotate about the specified axis
 * @param {Number} theta Specifies the angle of rotation.
 */
Quaternion.prototype.setToRotateAboutZ = function (theta) {
  // Compute the half angle
  var thetaOver2 = theta * .5
    ;

  // Set the values
  this.w = cos(thetaOver2);
  this.x = 0;
  this.y = 0;
  this.z = sin(thetaOver2);
};

/**
 * Setup the quaternion to rotate about the specified axis
 * @param {Vector3} axis Specifies the axis of rotation.
 * @param {Number} theta Specifies the angle of rotation.
 */
Quaternion.prototype.setToRotateAboutAxis = function (axis, theta) {
  // The axis of rotation must be normalized
  if (!(abs(axis.magnitude() - 1) < .01)) {
    throw new TypeError('The axis of rotation must be normalized');
  }

  // Compute the half angle and its sin
  var thetaOver2 = theta * .5
    , sinThetaOver2 = sin(thetaOver2)
    ;

  // Set the values
  this.w = cos(thetaOver2);
  this.x = axis.x * sinThetaOver2;
  this.y = axis.y * sinThetaOver2;
  this.z = axis.z * sinThetaOver2;
};

/**
 * Setup the quaternion to perform an object->inertial rotation, given the
 * orientation in Euler angle format
 *
 * See 10.6.5 for more information.
 * @param {EulerAngles} orientation Represents the rotation in Euler angle form.
 */
Quaternion.prototype.setToRotateObjectToInertial = function (orientation) {
  // Compute sine and cosine of the half angles
  var headingOver2 = orientation.heading * 0.5
    , pitchOver2 = orientation.pitch * 0.5
    , bankOver2 = orientation.bank * 0.5
    , sh = sin(headingOver2)
    , sp = sin(pitchOver2)
    , sb = sin(bankOver2)
    , ch = cos(headingOver2)
    , cp = cos(pitchOver2)
    , cb = cos(bankOver2)
    ;

  // Compute values
  this.w = ch * cp * cb + sh * sp * sb;
  this.x = ch * sp * cb + sh * cp * sb;
  this.y = -ch * sp * sb + sh * cp * cb;
  this.z = -sh * sp * cb + ch * cp * sb;
};

/**
 * Setup the quaternion to perform an inertial->object rotation, given the
 * orientation in Euler angle format
 *
 * See 10.6.5 for more information.
 * @param {EulerAngles} orientation Represents the rotation in Euler angle form.
 */
Quaternion.prototype.setToRotateInertialToObject = function (orientation) {
  // Compute sine and cosine of the half angles
  var headingOver2 = orientation.heading * 0.5
    , pitchOver2 = orientation.pitch * 0.5
    , bankOver2 = orientation.bank * 0.5
    , sh = sin(headingOver2)
    , sp = sin(pitchOver2)
    , sb = sin(bankOver2)
    , ch = cos(headingOver2)
    , cp = cos(pitchOver2)
    , cb = cos(bankOver2)
    ;

  // Compute values
  this.w = ch * cp * cb + sh * sp * sb;
  this.x = -ch * sp * cb - sh * cp * sb;
  this.y = ch * sp * sb - sh * cb * cp;
  this.z = sh * sp * cb - ch * cp * sb;
};

/**
 * "Normalize" a quaternion.  Note that normally, quaternions
 * are always normalized (within limits of numerical precision).
 * See section 10.4.6 for more information.
 *
 * This function is provided primarily to combat floating point "error
 * creep," which can occur when many successive quaternion operations
 * are applied.
 * Normalizes the quaternion to unit length.
 */
Quaternion.prototype.normalize = function () {
  // Compute magnitude of the quaternion
  var mag = sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z)
    ;

  // Check for bogus length, to protect against divide by zero
  if (mag > 0) {
    // Normalize it
    var oneOverMag = 1 / mag;
    this.w *= oneOverMag;
    this.x *= oneOverMag;
    this.y *= oneOverMag;
    this.z *= oneOverMag;
  } else {
    // Houston, we have a problem
//    throw new Error();
    // In a release build, just slam it to something
    this.identity();
  }
};

/**
 * Return the rotation angle theta
 * @return {Number} The angle component of the quaternion.
 */
Quaternion.prototype.getRotationAngle = function () {
  // Compute the half angle.  Remember that w = cos(theta / 2)
  var thetaOver2 = safeAcos(this.w)
    ;

  // Return the rotation angle
  return thetaOver2 * 2;
};

/**
 * Return the rotation axis
 * @return {Vector3} The axis component of the quaternion.
 */
Quaternion.prototype.getRotationAxis = function () {
  // Compute sin^2(theta/2).  Remember that w = cos(theta/2),
  // and sin^2(x) + cos^2(x) = 1
  var sinThetaOver2Sq = 1 - this.w * this.w
    ;

  // Protect against numerical imprecision
  if (sinThetaOver2Sq <= 0) {
    // Identity quaternion, or numerical imprecision.  Just
    // return any valid vector, since it doesn't matter
    return new Vector3(1, 0, 0);
  }

  // Compute 1 / sin(theta/2)
  var oneOverSinThetaOver2 = 1 / sqrt(sinThetaOver2Sq)
    ;

  // Return axis of rotation
  return new Vector3(
    this.x * oneOverSinThetaOver2,
    this.y * oneOverSinThetaOver2,
    this.z * oneOverSinThetaOver2
  );
};

/**
 * Quaternion dot product.  We use a nonmember function so we can
 * pass quaternion expressions as operands without having "funky syntax"
 *
 * See 10.4.10
 * @param {Quaternion} a Specifies the second factor.
 * @return {Number} The dot product of the two quaternions
 */
Quaternion.prototype.dotProduct = function (a) {
  return this.w * a.w + this.x * a.x + this.y * a.y + this.z * a.z;
};

/**
 * Quaternion cross product, which concatenates multiple angular
 * displacements.  The order of multiplication, from left to right,
 * corresponds to the order that the angular displacements are
 * applied.  This is backwards from the *standard* definition of
 * quaternion multiplication.  See section 10.4.8 for the rationale
 * behind this deviation from the standard.
 * @param {Quaternion} a Specifies the second factor.
 * @return {Quaternion} The cross product of the two quaternions
 */
Quaternion.prototype.crossProduct = function (a) {
  var result = new Quaternion()
    ;

  result.w = this.w * a.w - this.x * a.x - this.y * a.y - this.z * a.z;
  result.x = this.w * a.x + this.x * a.w + this.z * a.y - this.y * a.z;
  result.y = this.w * a.y + this.y * a.w + this.x * a.z - this.z * a.x;
  result.z = this.w * a.z + this.z * a.w + this.y * a.x - this.x * a.y;

  return result;
};

/**
 * Spherical linear interpolation.
 *
 * See 10.4.13
 * @param {Quaternion} q0 Specifies the initial quaternion.
 * @param {Quaternion} q1 Specifies the final quaternion.
 * @param {Number} t Specifies the parametric time.
 * @return {Quaternion} The interpolated quaternion at the given time.
 */
Quaternion.slerp = function (q0, q1, t) {
  // Check for out-of range parameter and return edge points if so
  if (t <= 0) return q0;
  if (t >= 1) return q1;

  // Compute "cosine of angle between quaternions" using dot product
  var cosOmega = Quaternion.prototype.dotProduct(q0, q1)
    ;

  // If negative dot, use -q1.  Two quaternions q and -q
  // represent the same rotation, but may produce
  // different slerp.  We chose q or -q to rotate using
  // the acute angle.
  var q1w = q1.w
    , q1x = q1.x
    , q1y = q1.y
    , q1z = q1.z
    ;
  if (cosOmega < 0) {
    q1w = -q1w;
    q1x = -q1x;
    q1y = -q1y;
    q1z = -q1z;
    cosOmega = -cosOmega;
  }

  // We should have two unit quaternions, so dot should be <= 1.0
  if (!(cosOmega < 1.1)) {
    throw new TypeError('We should have two unit quaternions, so dot should be <= 1.0');
  }

  // Compute interpolation fraction, checking for quaternions
  // almost exactly the same
  var k0, k1
    ;
  if (cosOmega > 0.9999) {
    // Very close - just use linear interpolation,
    // which will protect against a divide by zero
    k0 = 1 - t;
    k1 = t;
  } else {
    // Compute the sin of the angle using the
    // trig identity sin^2(omega) + cos^2(omega) = 1
    var sinOmega = sqrt(1 - cosOmega * cosOmega)
      ;

    // Compute the angle from its sin and cosine
    var omega = atan2(sinOmega, cosOmega)
      ;

    // Compute inverse of denominator, so we only have
    // to divide once
    var oneOverSinOmega = 1 / sinOmega
      ;

    // Compute interpolation parameters
    k0 = sin((1 - t) * omega) * oneOverSinOmega;
    k1 = sin(t * omega) * oneOverSinOmega;
  }

  // Interpolate
  var result = new Quaternion()
    ;
  result.x = k0 * q0.x + k1 * q1x;
  result.y = k0 * q0.y + k1 * q1y;
  result.z = k0 * q0.z + k1 * q1z;
  result.w = k0 * q0.w + k1 * q1w;

  // Return it
  return result;
};

/**
 * Compute the quaternion conjugate.  This is the quaternion
 * with the opposite rotation as the original quaternion.  See 10.4.7
 * @return {Quaternion} The conjugate of the quaternion.
 */
Quaternion.prototype.conjugate = function () {
  var result = new Quaternion()
    ;

  // Same rotation amount
  result.w = this.w;

  // Opposite axis of rotation
  result.x = -this.x;
  result.y = -this.y;
  result.z = -this.z;

  // Return it
  return result;
};

/**
 * Quaternion exponentiation.
 *
 * See 10.4.12
 * @param {Number} exponent Specifies the exponent.
 * @return {Quaternion} The quaternion raised to the exponent.
 */
Quaternion.prototype.pow = function (exponent) {
  // Check for the case of an identity quaternion.
  // This will protect against divide by zero
  if (abs(this.w) > .9999) {
    return this;
  }

  // Extract the half angle alpha (alpha = theta/2)
  var alpha = acos(this.w)
    ;

  // Compute new alpha value
  var newAlpha = alpha * exponent
    ;

  // Compute new w value
  var result = new Quaternion()
    ;
  result.w = cos(newAlpha);

  // Compute new xyz values
  var mult = sin(newAlpha) / sin(alpha)
    ;
  result.x = this.x * mult;
  result.y = this.y * mult;
  result.z = this.z * mult;

  // Return it
  return result;
};