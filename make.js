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
    , EXPORTS = [
      'Vector3',
      'EulerAngles',
      'Quaternion',
      'RotationMatrix',
      'Matrix4x3'
    ]
    , DST = 'lib'
    , NAME = 'math3'

    , sys = require('sys')
    , fs = require('fs')
    , path = require('path')
    , uglify = require('uglify-js')

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
        , exports = []
        , code
        ;
      FILES.forEach(function (filename) {
        var code = fs.readFileSync(path.join(SRC, filename), 'utf8')
          ;
        code = code.replace(/[\r\n]$/, '');
        codes.push(code);
      });
      EXPORTS.forEach(function (exp) {
        exports.push('this.' + exp + ' = ' + exp + ';');
      });
      codes.push(exports.join('\n'));

      code = codes.join('\n\n\n');
      fs.writeFileSync(path.join(DST, NAME + '.raw.js'), code);

      codes = code.split(/\r?\n/);
      codes.forEach(function (line, i) {
        if (line === '') {
          return;
        }
        codes[i] = '  ' + line;
      });
      code = [
        '(function () {',
        codes.join('\n'),
        '}).call(this);'
      ].join('\n');
      fs.writeFileSync(path.join(DST, NAME + '.js'), code);

      code = uglify.minify(code, {
        fromString: true
      }).code;
      fs.writeFileSync(path.join(DST, NAME + '.min.js'), code);

      sys.puts([Date.now(), ':', 'Compiled'].join(' '));
    }
    ;

  fs.watch(SRC, changed);
  compile();

})();