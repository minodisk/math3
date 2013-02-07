var Math = this.Math
  , kPi = Math.PI
  , k2Pi = kPi * 2
  , kPiOver2 = kPi / 2
  , k1OverPi = 1 / kPi
  , k1Over2Pi = 1 / kPiOver2
  , sin = Math.sin
  , cos = Math.cos
  , tan = Math.tan
  , asin = Math.asin
  , acos = Math.acos
  , atan = Math.atan
  , atan2 = Math.atan2
  , sqrt = Math.sqrt
  , abs = Math.abs
  , floor = Math.floor
  , ceil = Math.ceil
  , wrapPi = function (theta) {
    theta += kPi;
    theta -= floor(theta * k1Over2Pi) * k2Pi;
    theta -= kPi;
    return theta;
  }
  , safeAcos = function (theta) {
    if (theta <= -1) {
      return kPi;
    }
    if (theta >= 1) {
      return 0;
    }
    return acos(theta);
  }

  , float = this.parseFloat
  , int = function (str) {
    return this.parseInt(str, 10);
  }

  , EPSILON = 1e-6 //0.000001
  , RADIAN_PER_DEGREE = kPi / 180
  , RADIAN_PER_DEGREE_1_2 = RADIAN_PER_DEGREE / 2

  , toString = Object.prototype.toString
  , isString = function (obj) {
    return toString.call(obj) === '[object String]';
  }
  , isArray = Array.isArray || function (obj) {
    return toString.call(obj) === '[object Array]';
  }
  , isNumber = function (obj) {
    return toString.call(obj) === '[object Number]';
  }
  , isInt = function (obj) {
    return obj === obj >> 0;
  }
  ;
