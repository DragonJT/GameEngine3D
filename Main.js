
var JMath = {};

JMath.Clamp01 = function(f){
    if(f>1) return 1;
    if(f<0) return 0;
    return f;
}

var Color = {};

Color.ToVector3 = function(c){
    return {x:c.r, y:c.g, z:c.b};
}

var Triangle = {};

Triangle.PointInside = function(point, a, b, c){
    if(!Triangle.Valid(a,b,c)) return false;
    const v0 = Vector3.Subtract(b,a);
    const v1 = Vector3.Subtract(c,a);
    const v2 = Vector3.Subtract(point, a);
    
    // Compute dot products
    const dot00 = v0.x * v0.x + v0.y * v0.y + v0.z * v0.z;
    const dot01 = v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
    const dot02 = v0.x * v2.x + v0.y * v2.y + v0.z * v2.z;
    const dot11 = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
    const dot12 = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    
    // Compute barycentric coordinates
    const invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    const u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    const v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    
    // Check if the point is inside the triangle
    return u >= 0 && v >= 0 && u + v <= 1;
}

Triangle.Valid = function(a,b,c){
    if(Vector3.Equals(a,b)) return false;
    if(Vector3.Equals(b,c)) return false;
    if(Vector3.Equals(c,a)) return false;
    return true;
}

Triangle.Raycast = function(ray, a,b,c){
    if(!Triangle.Valid(a,b,c)) return undefined;
    const plane = Plane.CreateFrom3Points(a,b,c);
    const point = Plane.Raycast(plane, ray);
    if(!point) return undefined;
    if(Triangle.PointInside(point,a,b,c)) return point;
    return undefined;
}

var Quaternion = {};

Quaternion.AxisAngle = function(axis, angle){
    const length = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    const s = Math.sin(angle / 2);
    const cosHalfAngle = Math.cos(angle / 2);

    // Calculate the quaternion components
    const x = (axis.x / length) * s;
    const y = (axis.y / length) * s;
    const z = (axis.z / length) * s;
    const w = cosHalfAngle;

    return { w, x, y, z };
}

Quaternion.Identity = function(){
    return {x:0, y:0, z:0, w:1};
}

Quaternion.Euler = function(alpha, beta, gamma) {
    // Convert Euler angles to radians
    alpha = (alpha * Math.PI) / 180;
    beta = (beta * Math.PI) / 180;
    gamma = (gamma * Math.PI) / 180;
  
    const cosAlpha = Math.cos(alpha / 2);
    const sinAlpha = Math.sin(alpha / 2);
    const cosBeta = Math.cos(beta / 2);
    const sinBeta = Math.sin(beta / 2);
    const cosGamma = Math.cos(gamma / 2);
    const sinGamma = Math.sin(gamma / 2);
  
    const w = cosAlpha * cosBeta * cosGamma + sinAlpha * sinBeta * sinGamma;
    const x = sinAlpha * cosBeta * cosGamma - cosAlpha * sinBeta * sinGamma;
    const y = cosAlpha * sinBeta * cosGamma + sinAlpha * cosBeta * sinGamma;
    const z = cosAlpha * cosBeta * sinGamma - sinAlpha * sinBeta * cosGamma;
  
    return { w, x, y, z };
}

Quaternion.Multiply = function(q1, q2) {
    const w1 = q1.w;
    const x1 = q1.x;
    const y1 = q1.y;
    const z1 = q1.z;
  
    const w2 = q2.w;
    const x2 = q2.x;
    const y2 = q2.y;
    const z2 = q2.z;
  
    const w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    const x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    const y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    const z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
  
    return { w, x, y, z };
}

Quaternion.ToMatrix = function(quaternion){
    const w = quaternion.w;
    const x = quaternion.x;
    const y = quaternion.y;
    const z = quaternion.z;
    
    const xx = x * x;
    const yy = y * y;
    const zz = z * z;
    const xy = x * y;
    const xz = x * z;
    const yz = y * z;
    const wx = w * x;
    const wy = w * y;
    const wz = w * z;
    
    const matrix = [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0,
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0,
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0,
        0, 0, 0, 1];
    return matrix;
}

var Line = {};

Line.RayHitLine = function(line, ray){
    const lineDirection = Vector3.Normalize(Vector3.Subtract(line.b, line.a));
    var left = Vector3.Normalize(Vector3.Cross(lineDirection, Vector3.Normalize(Vector3.Subtract(ray.origin, line.a))));
    var normal = Vector3.Normalize(Vector3.Cross(left, lineDirection));
    const plane = {point:line.a, normal:normal};
    const intersect = Plane.Raycast(plane, ray);
    if(!intersect){
        return undefined;
    }
    return {distance:Line.DistanceToPoint(line, intersect), intersect:intersect, plane:plane};
}

Line.DistanceToPoint = function(line, point) {
    const lineStart = line.a;
    const lineEnd = line.b;
    const lineVec = {
        x: lineEnd.x - lineStart.x,
        y: lineEnd.y - lineStart.y,
        z: lineEnd.z - lineStart.z,
    };

    const pointToLineStart = {
        x: point.x - lineStart.x,
        y: point.y - lineStart.y,
        z: point.z - lineStart.z,
    };

    const lineLengthSquared = (lineVec.x * lineVec.x) + (lineVec.y * lineVec.y) + (lineVec.z * lineVec.z);

    if (lineLengthSquared === 0) {
        // The line segment is just a point; return the distance to that point
        return Math.sqrt(
        (pointToLineStart.x * pointToLineStart.x) +
        (pointToLineStart.y * pointToLineStart.y) +
        (pointToLineStart.z * pointToLineStart.z)
        );
    }

    const t = (
        (pointToLineStart.x * lineVec.x) +
        (pointToLineStart.y * lineVec.y) +
        (pointToLineStart.z * lineVec.z)
    ) / lineLengthSquared;

    if (t < 0) {
        // The closest point is the line segment start point
        return Math.sqrt(
        (pointToLineStart.x * pointToLineStart.x) +
        (pointToLineStart.y * pointToLineStart.y) +
        (pointToLineStart.z * pointToLineStart.z)
        );
    }

    if (t > 1) {
        // The closest point is the line segment end point
        const pointToEnd = {
        x: point.x - lineEnd.x,
        y: point.y - lineEnd.y,
        z: point.z - lineEnd.z,
        };
        return Math.sqrt(
        (pointToEnd.x * pointToEnd.x) +
        (pointToEnd.y * pointToEnd.y) +
        (pointToEnd.z * pointToEnd.z)
        );
    }

    // The closest point is on the line segment between start and end
    const closestPoint = {
        x: lineStart.x + t * lineVec.x,
        y: lineStart.y + t * lineVec.y,
        z: lineStart.z + t * lineVec.z,
    };

    const distance = Math.sqrt(
        (point.x - closestPoint.x) * (point.x - closestPoint.x) +
        (point.y - closestPoint.y) * (point.y - closestPoint.y) +
        (point.z - closestPoint.z) * (point.z - closestPoint.z)
    );

    return distance;
}

var Plane = {};

Plane.CreateFrom3Points = function(a,b,c){
    const ba = Vector3.Subtract(b,a);
    const ca = Vector3.Subtract(c,a);
    const normal = Vector3.Normalize(Vector3.Cross(ba, ca));
    return {point:a, normal:normal};
}

Plane.Raycast = function(plane, ray) {
    const dotProduct = ray.direction.x * plane.normal.x + ray.direction.y * plane.normal.y + ray.direction.z * plane.normal.z;
  
    if (Math.abs(dotProduct) < 1e-6) {
      return undefined;
    }

    const t = (
      (plane.point.x - ray.origin.x) * plane.normal.x +
      (plane.point.y - ray.origin.y) * plane.normal.y +
      (plane.point.z - ray.origin.z) * plane.normal.z
    ) / dotProduct;
  
    if (t < 0) {
      return undefined;
    }
    return {
      x: ray.origin.x + t * ray.direction.x,
      y: ray.origin.y + t * ray.direction.y,
      z: ray.origin.z + t * ray.direction.z,
    };
}

var Vector3 = {};

Vector3.Equals = function(a,b){
    const min = 0.000001;
    return Math.abs(b.x-a.x)<min && Math.abs(b.y-a.y)<min && Math.abs(b.z-a.z)<min;
}

Vector3.Clamp01 = function(v){
    return {x:JMath.Clamp01(v.x), y:JMath.Clamp01(v.y), z:JMath.Clamp01(v.z)};
}

Vector3.ToColor = function(v){
    return {r:v.x, g:v.y, b:v.z};
}

Vector3.Magnitude = function(v){
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); 
}

Vector3.Dot = function(a,b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3.Normal = function(a,b,c){
    const ba = Vector3.Subtract(b,a);
    const ca = Vector3.Subtract(c,a);
    return Vector3.Normalize(Vector3.Cross(ba, ca));
}

Vector3.Normalize = function(v) 
{ 
    const len = Vector3.Magnitude(v);
    return {x: v.x/len, y: v.y/len, z: v.z/len}; 
} 
 
Vector3.Cross = function(a,b) 
{ 
    return { 
        x: a.y * b.z - a.z * b.y, 
        y: a.z * b.x - a.x * b.z, 
        z: a.x * b.y - a.y * b.x 
    }; 
} 

Vector3.Subtract = function(a,b){
    return {x: a.x-b.x, y: a.y-b.y, z: a.z-b.z};
}

Vector3.Add = function(a,b){
    return {x:a.x+b.x, y:a.y+b.y, z:a.z+b.z};
}

Vector3.Multiply = function(a,b){
    return {x:a.x*b.x, y:a.y*b.y, z:a.z*b.z};
}

Vector3.MultiplyScalar = function(vector, scalar){
    return {x:vector.x*scalar, y:vector.y*scalar, z:vector.z*scalar};
}

Vector3.Clone = function(v){
    return {x:v.x, y:v.y, z:v.z};
}
 
//most of matrix from MDN
var Matrix = {};

Matrix.Identity = function(){
    return [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
}
//modified from scratchapixel
Matrix.Lookat = function(from, to, up) { 
    var forward = Vector3.Subtract(from, to); 
    forward = Vector3.Normalize(forward); 
    var right = Vector3.Cross(up, forward); 
    right = Vector3.Normalize(right); 
    const newup = Vector3.Cross(forward, right); 
 
    return [right.x,  right.y,  right.z, 0,  newup.x, newup.y, newup.z, 0, forward.x, forward.y, forward.z, 0, from.x, from.y, from.z, 1]; 
} 

Matrix.ComputeSimpleProjectionMatrix = function (scaleFactor) {
    return [
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
      scaleFactor,
      0,
      0,
      0,
      scaleFactor,
    ];
}

Matrix.Transform = function (normalMatrix, vector) {
    var x = vector.x*normalMatrix[0] + vector.y* normalMatrix[3] + vector.z*normalMatrix[6];
    var y = vector.x*normalMatrix[1] + vector.y*normalMatrix[4] + vector.z*normalMatrix[7];
    var z = vector.x*normalMatrix[2] + vector.y*normalMatrix[5] + vector.z*normalMatrix[8];
    return {x:x, y:y, z:z};
}

Matrix.MultiplyPoint = function (matrix, point) {
    var x = point.x*matrix[0] + point.y* matrix[4] + point.z*matrix[8] + matrix[12];
    var y = point.x*matrix[1] + point.y*matrix[5] + point.z*matrix[9] + matrix[13];
    var z = point.x*matrix[2] + point.y*matrix[6] + point.z*matrix[10] + matrix[14];
    var w = point.x*matrix[3] + point.y*matrix[7] + point.z*matrix[11] + matrix[15];
    return {x:x/w, y:y/w, z:z/w};
}

Matrix.Multiply = function (a, b) {
  
  // TODO - Simplify for explanation
  // currently taken from https://github.com/toji/gl-matrix/blob/master/src/gl-matrix/mat4.js#L306-L337
  
  var result = [];
  
  var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3],
      a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7],
      a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11],
      a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];

  // Cache only the current line of the second matrix
  var b0  = b[0], b1 = b[1], b2 = b[2], b3 = b[3];  
  result[0] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  result[1] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  result[2] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  result[3] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = b[4]; b1 = b[5]; b2 = b[6]; b3 = b[7];
  result[4] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  result[5] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  result[6] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  result[7] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = b[8]; b1 = b[9]; b2 = b[10]; b3 = b[11];
  result[8] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  result[9] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  result[10] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  result[11] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = b[12]; b1 = b[13]; b2 = b[14]; b3 = b[15];
  result[12] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  result[13] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  result[14] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  result[15] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  return result;
}

Matrix.MultiplyArray = function (matrices) {
  
  var inputMatrix = matrices[0];
  
  for(var i=1; i < matrices.length; i++) {
    inputMatrix = Matrix.Multiply(inputMatrix, matrices[i]);
  }
  
  return inputMatrix;
}

Matrix.Normal = function (matrix) {

  /*
    This function takes the inverse and then transpose of the provided
    4x4 matrix. The result is a 3x3 matrix. Essentially the translation
    part of the matrix gets removed.
  
    https://github.com/toji/gl-matrix
  */
  
  var a00 = matrix[0], a01 = matrix[1], a02 = matrix[2], a03 = matrix[3],
      a10 = matrix[4], a11 = matrix[5], a12 = matrix[6], a13 = matrix[7],
      a20 = matrix[8], a21 = matrix[9], a22 = matrix[10], a23 = matrix[11],
      a30 = matrix[12], a31 = matrix[13], a32 = matrix[14], a33 = matrix[15],

      b00 = a00 * a11 - a01 * a10,
      b01 = a00 * a12 - a02 * a10,
      b02 = a00 * a13 - a03 * a10,
      b03 = a01 * a12 - a02 * a11,
      b04 = a01 * a13 - a03 * a11,
      b05 = a02 * a13 - a03 * a12,
      b06 = a20 * a31 - a21 * a30,
      b07 = a20 * a32 - a22 * a30,
      b08 = a20 * a33 - a23 * a30,
      b09 = a21 * a32 - a22 * a31,
      b10 = a21 * a33 - a23 * a31,
      b11 = a22 * a33 - a23 * a32,

      // Calculate the determinant
      det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

  if (!det) { 
    return null; 
  }
  det = 1.0 / det;
  
  var result = []

  result[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
  result[1] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
  result[2] = (a10 * b10 - a11 * b08 + a13 * b06) * det;

  result[3] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
  result[4] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
  result[5] = (a01 * b08 - a00 * b10 - a03 * b06) * det;

  result[6] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
  result[7] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
  result[8] = (a30 * b04 - a31 * b02 + a33 * b00) * det;

  return result;
}

Matrix.RotateX = function (a) {
  
  var cos = Math.cos;
  var sin = Math.sin;
  
  return [
       1,       0,        0,     0,
       0,  cos(a),  -sin(a),     0,
       0,  sin(a),   cos(a),     0,
       0,       0,        0,     1
  ];
}

Matrix.RotateY = function (a) {

  var cos = Math.cos;
  var sin = Math.sin;
  
  return [
     cos(a),   0, sin(a),   0,
          0,   1,      0,   0,
    -sin(a),   0, cos(a),   0,
          0,   0,      0,   1
  ];
}

Matrix.RotateZ = function (a) {

  var cos = Math.cos;
  var sin = Math.sin;
  
  return [
    cos(a), -sin(a),    0,    0,
    sin(a),  cos(a),    0,    0,
         0,       0,    1,    0,
         0,       0,    0,    1
  ];
}

Matrix.Translate = function (x, y, z) {
	return [
	    1,    0,    0,   0,
	    0,    1,    0,   0,
	    0,    0,    1,   0,
	    x,    y,    z,   1
	];
}

Matrix.Scale = function (w, h, d) {
	return [
	    w,    0,    0,   0,
	    0,    h,    0,   0,
	    0,    0,    d,   0,
	    0,    0,    0,   1
	];
}

Matrix.Perspective = function (fieldOfViewInRadians, aspectRatio, near, far) {
  
  // Construct a perspective matrix
  
  /*
     Field of view - the angle in radians of what's in view along the Y axis
     Aspect Ratio - the ratio of the canvas, typically canvas.width / canvas.height
     Near - Anything before this point in the Z direction gets clipped (resultside of the clip space)
     Far - Anything after this point in the Z direction gets clipped (outside of the clip space)
  */
  
  var f = 1.0 / Math.tan(fieldOfViewInRadians / 2);
  var rangeInv = 1 / (near - far);
 
  return [
    f / aspectRatio, 0,                          0,   0,
    0,               f,                          0,   0,
    0,               0,    (near + far) * rangeInv,  -1,
    0,               0,  near * far * rangeInv * 2,   0
  ];
}

Matrix.Orthographic = function(left, right, bottom, top, near, far) {
  
  // Each of the parameters represents the plane of the bounding box
  
  var lr = 1 / (left - right);
  var bt = 1 / (bottom - top);
  var nf = 1 / (near - far);
	
  var row4col1 = (left + right) * lr;
  var row4col2 = (top + bottom) * bt;
  var row4col3 = (far + near) * nf;
  
  return [
     -2 * lr,        0,        0, 0,
           0,  -2 * bt,        0, 0,
           0,        0,   2 * nf, 0,
    row4col1, row4col2, row4col3, 1
  ];
}

Matrix.Normalize = function( vector ) {
  
  // A utility function to make a vector have a length of 1
  
  var length = Math.sqrt(
    vector[0] * vector[0] +
    vector[1] * vector[1] +
    vector[2] * vector[2]
  )
  
  return [
    vector[0] / length,
    vector[1] / length,
    vector[2] / length
  ]
}

Matrix.Invert = function( matrix ) {
	
	// Adapted from: https://github.com/mrdoob/three.js/blob/master/src/math/Matrix4.js
	
	// Performance note: Try not to allocate memory during a loop. This is done here
	// for the ease of understanding the code samples.
	var result = [];

	var n11 = matrix[0], n12 = matrix[4], n13 = matrix[ 8], n14 = matrix[12];
	var n21 = matrix[1], n22 = matrix[5], n23 = matrix[ 9], n24 = matrix[13];
	var n31 = matrix[2], n32 = matrix[6], n33 = matrix[10], n34 = matrix[14];
	var n41 = matrix[3], n42 = matrix[7], n43 = matrix[11], n44 = matrix[15];

	result[ 0] = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44;
	result[ 4] = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44;
	result[ 8] = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44;
	result[12] = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;
	result[ 1] = n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44;
	result[ 5] = n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44;
	result[ 9] = n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44;
	result[13] = n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34;
	result[ 2] = n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44;
	result[ 6] = n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44;
	result[10] = n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44;
	result[14] = n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34;
	result[ 3] = n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43;
	result[ 7] = n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43;
	result[11] = n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43;
	result[15] = n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33;

	var determinant = n11 * result[0] + n21 * result[4] + n31 * result[8] + n41 * result[12];

	if ( determinant === 0 ) {
		throw new Error("Can't invert matrix, determinant is 0");
	}
	
	for( var i=0; i < result.length; i++ ) {
		result[i] /= determinant;
	}

	return result;
}

const vertexCode = `#version 300 es
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec3 normal;
out vec3 fragPos;
out vec3 color;

void main() {
    gl_Position = projection * view * model * vec4(vertexPosition, 1);
    fragPos = vec3(model * vec4(vertexPosition, 1.0));
    normal = mat3(transpose(inverse(model))) * vertexNormal;
    color = vertexColor;  
}`;

const fragmentCode = `#version 300 es
precision highp float;

in vec3 normal;
in vec3 fragPos;
in vec3 color;

uniform vec3 viewPos;
uniform vec3 lightPos;
uniform float ambientStrength;
uniform vec3 tint;

out vec4 FragColor;

void main() {
    vec3 lightColor = vec3(1,1,1);
    float specularStrength = 0.5;

    vec3 ambient = ambientStrength * lightColor;

    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPos - fragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;  

    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * lightColor;  

    vec3 result = (ambient + diffuse + specular) * color * tint;
    FragColor = vec4(result, 1.0);
}`;

var Mesh = {};

Mesh.MultiplyMatrix = function(mesh, matrix){
    var normalMatrix = Matrix.Normal(matrix);
    var vertices = [];
    var faces = [];
    for(var v of mesh.vertices){
        var vertex = {};
        vertex.position = Matrix.MultiplyPoint(matrix, v.position);
        if(v.normal){
            vertex.normal = Matrix.Transform(normalMatrix, v.normal);
        }
        vertices.push(vertex);
    }
    for(var f of mesh.faces){
        faces.push(f);
    }
    return {vertices:vertices, faces:faces};
}

Mesh.Join = function(a,b){
    var vertices = [];
    for(var v of a.vertices){
        vertices.push(v);
    }
    var indexCount = vertices.length;
    for(var v of b.vertices){
        vertices.push(v);
    }
    var faces = [];
    for(var f of a.faces){
        faces.push(f);
    }
    for(var f of b.faces){
        var indices = [];
        for(var i of f.indices){
            indices.push(i + indexCount);
        }
        faces.push({indices:indices, color:f.color, useFaceNormals:f.useFaceNormals});
    }
    return {vertices:vertices, faces:faces};
}

Mesh.JoinArray = function(array){
    var mesh = {vertices:[], faces:[]};
    for(var m of array){
        mesh = Mesh.Join(mesh, m);
    }
    return mesh;
}

Mesh.Cube = function(color){
    return {
        vertices:[
            {position:{x:-1, y:-1, z:-1}},
            {position:{x:1, y:-1, z:-1}},
            {position:{x:1, y:1, z:-1}},
            {position:{x:-1, y:1, z:-1}},
            {position:{x:-1, y:-1, z:1}},
            {position:{x:1, y:-1, z:1}},
            {position:{x:1, y:1, z:1}},
            {position:{x:-1, y:1, z:1}},
        ],
        faces:[
            {indices:[3,2,1,0], color:color},
            {indices:[4,5,6,7], color:color},
            {indices:[0,1,5,4], color:color},
            {indices:[1,2,6,5], color:color},
            {indices:[2,3,7,6], color:color},
            {indices:[3,0,4,7], color:color},
        ]
    };
}

Mesh.Sphere = function(segments, color){
    var vertices = [];
    var delta = (2*Math.PI)/segments;
    var angley = 0;
    for(var i1=0;i1<segments;i1++){
        var anglex=0;
        var radius = Math.sin(angley);
        var y=Math.cos(angley);
        for(var i2=0;i2<segments;i2++){
            var position = {x:Math.cos(anglex)*radius, y:y, z:Math.sin(anglex)*radius};
            vertices.push({position:position, normal:position});
            anglex+=delta;
        }
        angley+=delta;
    }
    var faces = [];
    for(var i1=0;i1<segments-1;i1++){
        var currenty = i1*segments;
        var nexty = (i1+1)*segments;
        for(var i2=0;i2<segments;i2++){
            var nextx = (i2+1)%segments;
            faces.push({indices:[i2+currenty,nextx+currenty,nextx+nexty,i2+nexty], color:color});
        }
    }
    return {vertices:vertices, faces:faces};
}

Mesh.Cylinder = function(segments, color, bottom=0, top=1, bottomRadius=1, topRadius=1){
    var vertices = [];
    var delta = (2*Math.PI)/segments;
    var angle = 0;
    for(var i2=0;i2<segments;i2++){
        var normal = {x:Math.cos(angle), y:0, z:Math.sin(angle)};
        var bottomPositions = {x:Math.cos(angle)*bottomRadius, y:bottom, z:Math.sin(angle)*bottomRadius};
        var topPositions = {x:Math.cos(angle)*topRadius, y:top, z:Math.sin(angle)*topRadius};
        vertices.push({position:bottomPositions, normal:normal});
        vertices.push({position:topPositions, normal:normal});
        angle+=delta;
    }
    var faces = [];
    for(var i=0;i<segments;i++){
        var i2 = (i+1)%segments;
        faces.push({indices:[i*2, i*2+1, i2*2+1, i2*2], color:color});
    }
    if(bottomRadius){
        var bottomIndices = [];
        for(var i=0;i<segments;i++){
            bottomIndices.push(i*2);
        }
        faces.push({indices:bottomIndices, useFaceNormals:true, color:color});
    }
    if(topRadius){
        var topIndices = [];
        for(var i=segments-1;i>=0;i--){
            topIndices.push(i*2+1);
        }
        faces.push({indices:topIndices, useFaceNormals:true, color:color});
    }
    return {vertices:vertices, faces:faces};
}

Mesh.Cone = function(segments, color){
    return Mesh.Cylinder(segments, color, 0, 1, 1, 0);
}

Mesh.Arrow = function(segments, color, lineTop, tipTop, lineRadius, tipRadius){
    var line = Mesh.Cylinder(segments, color, 0, lineTop, lineRadius, lineRadius);
    var tip = Mesh.Cylinder(segments, color, lineTop, tipTop, tipRadius, 0);
    return Mesh.Join(line, tip);
}

Mesh.Gizmo = function(segments, scale){
    var gizmoY = Mesh.Arrow(segments, {r:0, g:1, b:0}, 0.8*scale, scale, 0.04*scale, 0.1*scale);
    var gizmoZ = Mesh.MultiplyMatrix(Mesh.Arrow(segments, {r:0, g:0, b:1},0.8*scale, scale, 0.04*scale, 0.1*scale), Matrix.RotateX(Math.PI*1.5));
    var gizmoX = Mesh.MultiplyMatrix(Mesh.Arrow(segments, {r:1, g:0, b:0},0.8*scale, scale, 0.04*scale, 0.1*scale), Matrix.RotateZ(Math.PI*0.5));
    return Mesh.JoinArray([gizmoY, gizmoZ, gizmoX]);
}

Mesh.CalcData = function(mesh){
    mesh.positionData = [];
    mesh.normalData = [];
    mesh.colorData = [];
    mesh.indexData = [];
    var vertexID = 0;
    for(var face of mesh.faces){
        var normal = Vector3.Normal(
            mesh.vertices[face.indices[0]].position, 
            mesh.vertices[face.indices[1]].position, 
            mesh.vertices[face.indices[2]].position);

        for(var vi of face.indices){
            const v = mesh.vertices[vi];
            mesh.positionData.push(v.position.x);
            mesh.positionData.push(v.position.y);
            mesh.positionData.push(v.position.z);
            if(face.useFaceNormals || !v.normal){
                mesh.normalData.push(normal.x);
                mesh.normalData.push(normal.y);
                mesh.normalData.push(normal.z);
            }
            else{
                mesh.normalData.push(v.normal.x);
                mesh.normalData.push(v.normal.y);
                mesh.normalData.push(v.normal.z);
            }
            if(!v.color){
                mesh.colorData.push(face.color.r);
                mesh.colorData.push(face.color.g);
                mesh.colorData.push(face.color.b);
            }
            else{
                mesh.colorData.push(v.color.r);
                mesh.colorData.push(v.color.g);
                mesh.colorData.push(v.color.b);
            }
        }
        for(var i=2;i<face.indices.length;i++){
            mesh.indexData.push(vertexID);
            mesh.indexData.push(vertexID+i-1);
            mesh.indexData.push(vertexID+i);
        }
        vertexID+=face.indices.length;
    }

    mesh.positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(mesh.positionData), gl.STATIC_DRAW);

    mesh.normalBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.normalBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(mesh.normalData), gl.STATIC_DRAW);

    mesh.colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(mesh.colorData), gl.STATIC_DRAW);

    mesh.indexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mesh.indexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(mesh.indexData), gl.STATIC_DRAW);
}

Mesh.Render = function(mesh, program, model, camera, lightPos, tint, ambientStrength){
    gl.useProgram(program);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.positionBuffer);
    const vertexPosition = gl.getAttribLocation(program, "vertexPosition");
    gl.enableVertexAttribArray(vertexPosition);
    gl.vertexAttribPointer(vertexPosition, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.normalBuffer);
    const vertexNormal = gl.getAttribLocation(program, "vertexNormal");
    gl.enableVertexAttribArray(vertexNormal);
    gl.vertexAttribPointer(vertexNormal, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.colorBuffer);
    const vertexColor = gl.getAttribLocation(program, "vertexColor");
    gl.enableVertexAttribArray(vertexColor);
    gl.vertexAttribPointer(vertexColor, 3, gl.FLOAT, false, 0, 0);

    gl.uniformMatrix4fv(gl.getUniformLocation(program, 'model'), false, model);
    gl.uniformMatrix4fv(gl.getUniformLocation(program, 'view'), false, camera.view);
    gl.uniformMatrix4fv(gl.getUniformLocation(program, 'projection'), false, camera.projection);
    const v = camera.viewPos;
    gl.uniform3f(gl.getUniformLocation(program, 'viewPos'), v.x, v.y, v.z);
    gl.uniform3f(gl.getUniformLocation(program, 'lightPos'), lightPos.x, lightPos.y, lightPos.z);
    gl.uniform1f(gl.getUniformLocation(program, 'ambientStrength'), ambientStrength);
    gl.uniform3f(gl.getUniformLocation(program, 'tint'), tint.r, tint.g, tint.b);

    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mesh.indexBuffer);
    gl.drawElements(gl.TRIANGLES,mesh.indexData.length, gl.UNSIGNED_SHORT, 0);    
}

Mesh.Raycast = function(ray, mesh, matrix){
    var minDist = undefined;
    for(var face of mesh.faces){
        for(var i=2;i<face.indices.length;i++){
            const v1 = mesh.vertices[face.indices[0]].position;
            const v2 = mesh.vertices[face.indices[i-1]].position;
            const v3 = mesh.vertices[face.indices[i]].position;
            const a = Matrix.MultiplyPoint(matrix, v1);
            const b = Matrix.MultiplyPoint(matrix, v2);
            const c = Matrix.MultiplyPoint(matrix, v3);
            const point = Triangle.Raycast(ray, a, b, c);
            if(point){
                const dist = Vector3.Magnitude(Vector3.Subtract(point, ray.origin));
                if(!minDist || dist<minDist){
                    minDist = dist;
                }
            }
        }
    }
    return minDist;
}

var Camera = {};

Camera.Resize = function(camera){
    camera.projection = Matrix.Perspective(Math.PI*0.5, gl.canvas.width/gl.canvas.height, 1, 100);
}

Camera.LookAt = function(camera, viewPos, target){
    camera.projection = Matrix.Perspective(Math.PI*0.5, gl.canvas.width/gl.canvas.height, 1, 100);
    camera.viewPos = viewPos;
    camera.view = Matrix.Invert(Matrix.Lookat(viewPos, target, {x:0, y:1, z:0}));
}

Camera.ViewportToWorldPoint = function(camera, v){
    const inverseCamera = Matrix.Invert(Matrix.Multiply(camera.projection, camera.view));
    return Matrix.MultiplyPoint(inverseCamera, v);
}

Camera.ScreenToRay = function(camera, sx, sy){
    var vx = sx/gl.canvas.width*2-1;
    var vy = (gl.canvas.height-sy)/gl.canvas.height*2-1;
    const v1 = {x:vx, y:vy, z:-1};
    const v2 = {x:vx, y:vy, z:1};
    const inverseCamera = Matrix.Invert(Matrix.Multiply(camera.projection, camera.view));
    const a = Matrix.MultiplyPoint(inverseCamera, v1);
    const b = Matrix.MultiplyPoint(inverseCamera, v2);
    const ray = {origin:a, direction:Vector3.Normalize(Vector3.Subtract(b, a))};
    return ray;
}

var PivotCamera = {};

PivotCamera.LookAt = function(camera){
    const viewPos = {
        x:camera.pivot.x + Math.sin(camera.angleY)*camera.radius, 
        y:camera.pivot.y + camera.height, 
        z:camera.pivot.z + Math.cos(camera.angleY)*camera.radius
    };
    Camera.LookAt(camera, viewPos, camera.pivot);
}

PivotCamera.Create = function(camera){
    camera.angleY = 0;
    camera.height = 5;
    camera.radius = 10;
    camera.pivot = {x:0, y:0, z:0};
    PivotCamera.LookAt(camera);
}

PivotCamera.MouseDrag = function(camera){
    if(keys['Meta']){
        camera.angleY += (mouse.current.x - mouse.last.x)*0.01;
        camera.height -= (mouse.current.y - mouse.last.y)*0.2;
        PivotCamera.LookAt(camera);
    }

}

var Gizmo = {};

Gizmo.Select = function(ray, axis, scale, key){
    const matrix = Quaternion.ToMatrix(selected.rotation);
    const direction = Matrix.MultiplyPoint(matrix, axis);
    const line = {a:selected.position, b:Vector3.Add(selected.position, Vector3.MultiplyScalar(direction, scale))};
    const intersect = Line.RayHitLine(line, ray);
    if(intersect && intersect.distance<0.4){
        dragging={type:'GIZMO', startPosition:selected.position, startScale:selected.scale, startRotation:selected.rotation, startColor:selected.color, 
            startIntersect:intersect.intersect, plane:intersect.plane, axis:axis, direction:direction, key:key};
        if(key == 'd'){
            selected = JSON.parse(JSON.stringify(selected));
            objects.push(selected);
        }
        return true;
    }
    return false;
}

Gizmo.KeyDown = function(e){
    if(!selected){
        return;
    }
    const ray = Camera.ScreenToRay(camera, mouse.current.x, mouse.current.y);
    if(Gizmo.Select(ray, {x:1, y:0, z:0}, 5, e.key)) return;
    if(Gizmo.Select(ray, {x:0, y:1, z:0}, 5, e.key)) return;
    if(Gizmo.Select(ray, {x:0, y:0, z:1}, 5, e.key)) return;
}

Gizmo.MouseMove = function(){
    if(dragging && dragging.type == 'GIZMO'){
        const ray = Camera.ScreenToRay(camera, mouse.current.x, mouse.current.y);
        const intersect = Plane.Raycast(dragging.plane, ray);
        if(!intersect){
            return;
        }
        if(dragging.key == 't' || dragging.key == 'd'){
            const dot = Vector3.Dot(dragging.direction, Vector3.Subtract(intersect, dragging.startIntersect));
            selected.position = Vector3.Add(dragging.startPosition, Vector3.MultiplyScalar(dragging.direction, dot));
        }
        else if(dragging.key == 's'){
            const dot = Vector3.Dot(dragging.direction, Vector3.Subtract(intersect, dragging.startIntersect));
            selected.scale = Vector3.Add(dragging.startScale, Vector3.MultiplyScalar(dragging.axis, dot));
        }
        else if(dragging.key == 'r'){
            const dot = Vector3.Dot(dragging.direction, Vector3.Subtract(intersect, dragging.startIntersect)) * 20;
            const q = Quaternion.AxisAngle(dragging.direction, dot*0.03);
            selected.rotation = Quaternion.Multiply(dragging.startRotation, q);
        }
        else if(dragging.key == 'c'){
            const dot = Vector3.Dot(dragging.direction, Vector3.Subtract(intersect, dragging.startIntersect));
            const c = Vector3.Add(Color.ToVector3(dragging.startColor), Vector3.MultiplyScalar(dragging.axis, dot));
            selected.color = Vector3.ToColor(Vector3.Clamp01(c));
        }
    }
}

Gizmo.Render = function(gizmo, camera, lightPos){
    const p = gizmo.position;
    Mesh.Render(gizmo.mesh, program, Matrix.Translate(p.x, p.y, p.z), camera, lightPos, 0.6);
}

function Save(){
    var a = document.createElement("a");
    a.href = window.URL.createObjectURL(new Blob([JSON.stringify(objects)], {type: "text/plain"}));
    a.download = "save.txt";
    a.click();
}

function CreateShader(shaderType, sourceCode){
    const shader = gl.createShader(shaderType);
    gl.shaderSource(shader, sourceCode);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw gl.getShaderInfoLog(shader);
    }
    return shader;
}

function CreateCanvas(parent){
    var canvas = document.createElement('canvas');
    parent.appendChild(canvas);
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    var gl = canvas.getContext('webgl2');
    return gl;
}

const gl = CreateCanvas(document.body);
document.body.style.margin = '0px';
document.body.style.overflow = 'hidden';

if (!gl) throw "WebGL2 not supported";

const program = gl.createProgram();
gl.attachShader(program, CreateShader(gl.VERTEX_SHADER, vertexCode));
gl.attachShader(program, CreateShader(gl.FRAGMENT_SHADER, fragmentCode));
gl.linkProgram(program);
if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
  throw gl.getProgramInfoLog(program);
}
gl.useProgram(program);

var selected = undefined;
var meshID = 0;
var meshes = []
var objects = [];
var camera = {};
var mouse = {};
var dragging = undefined;
var gizmo = Mesh.Gizmo(20, 5);
var keys = {};

Mesh.CalcData(gizmo);

meshes.push(Mesh.Cube({r:1, g:1, b:1}));
meshes.push(Mesh.Sphere(20, {r:1, g:1, b:1}));
meshes.push(Mesh.Cylinder(20, {r:1, g:1, b:1}));
meshes.push(Mesh.Cone(20, {r:1, g:1, b:1}));

for(var mesh of meshes){
    Mesh.CalcData(mesh);
}

function CreateObject(){
    selected = {position:camera.pivot, scale:{x:1, y:1, z:1}, color:{r:0.5, g:0.5, b:0.5}, rotation:Quaternion.Identity(), meshID:meshID};
    objects.push(selected);
}

PivotCamera.Create(camera);

function Resize(){
    gl.canvas.width = window.innerWidth;
    gl.canvas.height = window.innerHeight;
    Camera.Resize(camera);
}

function ObjectMatrix(o){
    return Matrix.MultiplyArray([
        Matrix.Translate(o.position.x, o.position.y, o.position.z),
        Quaternion.ToMatrix(o.rotation),
        Matrix.Scale(o.scale.x, o.scale.y, o.scale.z)]);
}

function RaycastObjects(){
    var minObject = undefined;
    var minDist = undefined;
    var ray = Camera.ScreenToRay(camera, mouse.current.x, mouse.current.y);
    for(var o of objects){
        const matrix = ObjectMatrix(o);
        const dist = Mesh.Raycast(ray, meshes[o.meshID], matrix);
        if(dist && (!minDist || dist<minDist)){
            minObject = o;
            minDist = dist;
        }
    }
    return minObject;
}

function MouseDown(){
    selected = RaycastObjects();
}

function KeyDown(e){
    if(e.key == ' '){
        CreateObject();
    }
    if(e.key == ','){
        meshID--;
        if(meshID<0) meshID=meshes.length-1;
        selected.meshID=meshID;
    }
    if(e.key == '.'){
        meshID++;
        if(meshID>=meshes.length) meshID=0;
        selected.meshID=meshID;
    }
    if(e.key == 'f'){
        camera.pivot = selected.position;
        PivotCamera.LookAt(camera);
    }
    if(e.key == 'Enter'){
        Save();
    }
    if(e.key == 'Backspace'){
        if(selected){
            const index = objects.indexOf(selected);
            objects.splice(index, 1);
            selected = undefined;
        }
    }
    if(!keys[e.key]){
        Gizmo.KeyDown(e);
    }
    keys[e.key] = true;
}

function KeyUp(e){
    keys[e.key] = undefined;
    dragging = undefined;
}

function MouseMove(e){
    const rect = gl.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top; 
    mouse.last = mouse.current;
    mouse.current = {x:mx, y:my};
    PivotCamera.MouseDrag(camera);
    Gizmo.MouseMove();
}

function Update(){
    gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);
    gl.cullFace(gl.BACK);

    gl.clearColor(0, 0, 0, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    const lightPos = {x:10, y:20, z:5};

    for(var o of objects){
        Mesh.Render(meshes[o.meshID], program, ObjectMatrix(o), camera, lightPos, o.color, 0.1);
    }
    gl.clear(gl.DEPTH_BUFFER_BIT);
    if(selected){
        var matrix = Matrix.Multiply(
            Matrix.Translate(selected.position.x, selected.position.y, selected.position.z), 
            Quaternion.ToMatrix(selected.rotation));
        Mesh.Render(gizmo, program, matrix, camera, lightPos, {r:1, g:1, b:1}, 0.4);
    }
    requestAnimationFrame(Update);
}

var now = 0;
Update();
addEventListener('mousedown', MouseDown);
addEventListener('mousemove', MouseMove);
addEventListener('keydown', KeyDown);
addEventListener('keyup', KeyUp);
addEventListener('resize', Resize);