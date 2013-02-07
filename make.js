(function () {
  'use strict';

  var SRC = 'src'
    , FILES = [
      'MathUtil.js',
      'Vector3.js',
      'EulerAngles.js',
      'Quaternion.js',
      'RotationMatrix.js',
      'Matrix4x3.js'
    ]
    , DST = 'lib'
    , NAME = 'math3'

    , fs = require('fs')
    , path = require('path')

    , changed = (function () {
      var id
        ;
      return function () {
        clearTimeout(id);
        id = setTimeout(function () {
          compile()
        }, 100);
      };
    })()
    , compile = function () {
      var codes = []
        , code
        ;
      FILES.forEach(function (filename) {
        codes.push(fs.readFileSync(path.join(SRC, filename), 'utf8'));
      });

      codes.unshift('(function () {');
      codes.push('})();');
      code = codes.join('\n\n\n');

      fs.writeFileSync(path.join(DST, NAME + '.js'), code);
    }
    ;

  fs.watch(SRC, changed);
  compile();

})();