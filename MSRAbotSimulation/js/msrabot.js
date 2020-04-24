//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
MSRAbot = function (app)
{
    this.app = app;
    this.objBase = null;
    this.objects = [];
    this.path = null;
    this.animations = [];

    this.staticElbow = true;
    this.staticElbowAngleLeft = -(Math.PI/2) - 0.25;
    this.staticElbowAngleRight = -(Math.PI/2) + 0.25;

    this.groupElbowLeft = null;
    this.groupElbowRight = null;

    this.meshHelperLeft = null;
    this.meshHelperRight = null;
    this.helperMaterial = new THREE.MeshLambertMaterial({ color: 0xffff00, side: THREE.DoubleSide });

    this.doneInitializing = false;
}

MSRAbot.prototype.constructor = MSRAbot;

//--------------------------------------------------------------------------------------------
MSRAbot.prototype.initialize = function initMSRAbot(vReference)
{
    //
    // see https://threejs.org/docs/index.html#examples/en/loaders/GLTFLoader
    //
    var scope = this;

    this.materials = [
        new THREE.MeshPhongMaterial({ color: 0xEBEAE2, opacity: 0.6, transparent: true, flatShading: true }),
        new THREE.MeshPhongMaterial({ color: 0x87B5C9, opacity: 0.6, transparent: true, flatShading: true  }),
        new THREE.MeshPhongMaterial({ color: 0x404040, opacity: 0.6, transparent: true, flatShading: true  }),
        new THREE.MeshPhongMaterial({ color: 0xff0000, opacity: 0.6, transparent: true, flatShading: true  }),
        new THREE.MeshPhongMaterial({ color: 0x00ff00, opacity: 0.6, transparent: true, flatShading: true  }),
        new THREE.MeshPhongMaterial({ color: 0x0000ff, opacity: 0.6, transparent: true, flatShading: true  }),
        new THREE.MeshPhongMaterial({ color: 0xfff0f0, opacity: 0.6, transparent: true, flatShading: true  })
    ];

    var loader = new THREE.GLTFLoader().setPath('./models/');

    loader.load('MSRAbot.v0.7.glb', function (gltf) {
        gltf.scene.traverse(function (child) {
            var pn = child.parent ? (child.parent.name) : "";

            scope.objects[child.name] = child;

            if (child.name == 'Base') {
                scope.objBase = child;
                child.position.set(0, vReference.y, 0);
            }

            if (child.name == 'Hood')
                child.material = scope.materials[1];
            else if (child.name == 'Eye')
                child.material = scope.materials[2];
            else if (child.name == 'SensorPlate')
                child.material = scope.materials[1];
            else if (child.name == 'MicrophonePlate')
                child.material = scope.materials[2];
            else
                child.material = scope.materials[0];

            if (child.isMesh) {
                child.castShadow = true;
                child.receiveShadow = true;
            }
        });

        scope.initializeModelObjects();

        scope.setShowHelpers(scope.app.params.showHelpers);

        //
        // done. fire init complete event
        scope.doneInitializing = true;

        if (scope.app.eventDoneInit != null)
            document.dispatchEvent(scope.app.eventDoneInit);
    });
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.initializeModelObjects = function ()
{
    if (this.objBase) {
        this.objBase.updateMatrix();
        this.objBase.updateMatrixWorld(true);

        this.app.scene.add(this.objBase);
    }

    var strExpectedObjects = [ 'Shoulder_Left', 'Upper_Arm_Left', 'Elbow_Left', 'Shoulder_Right', 'Upper_Arm_Right', 'Elbow_Right' ];

    for (var i = 0; i < strExpectedObjects.length; i++) {
        var name = strExpectedObjects[i];
        var obj = this.objects[name];
        if (!isObject(obj)) {
            console.error("The MSRAbot 3D model does not contain required object of name '" + name + "'!");
            window.alert("The MSRAbot 3D model does not contain required object of name '" + name + "'!");
        }
    }

    //
    // calculate arm segement lengths
    {
        var pos1, pos2;
        var arm_length = 50;

        //
        // left arm
        pos1 = this.getWorldPosition("Upper_Arm_Left");
        pos2 = this.getWorldPosition("Hand_Left"); // "Elbow_Left");

        this.seg1_left_len = pos2.sub(pos1).length();
        this.seg2_left_len = arm_length;

        //
        // right arm
        pos1 = this.getWorldPosition("Upper_Arm_Right");
        pos2 = this.getWorldPosition("Hand_Right"); // "Elbow_Right");

        this.seg1_right_len = pos2.sub(pos1).length();
        this.seg2_right_len = arm_length;
    }

    this.initializeHelpers();
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.initializeHelpers = function ()
{
    //
    // elbow helpers
    this.groupElbowLeft = new THREE.Group();
    this.app.scene.add(this.groupElbowLeft);
    this.groupElbowLeft.parent = this.objects['Upper_Arm_Left'];
    this.groupElbowLeft.position.x = 0.0;
    this.groupElbowLeft.position.y = 0.0;
    this.groupElbowLeft.position.z = this.seg1_left_len;

    this.groupElbowRight = new THREE.Group();
    this.app.scene.add(this.groupElbowRight);
    this.groupElbowRight.parent = this.objects['Upper_Arm_Right'];
    this.groupElbowRight.position.x = 0.0;
    this.groupElbowRight.position.y = 0.0;
    this.groupElbowRight.position.z = this.seg1_right_len;

    //
    // spheres
    var sphereGeometry = new THREE.SphereBufferGeometry(8, 16, 16);
    var sphereMaterial1 = new THREE.MeshPhongMaterial({ specular: 0x333333, shininess: 5, color: 0x0000ff });
    this.helperSphereL1 = new THREE.Mesh(sphereGeometry, sphereMaterial1);
    this.app.scene.add(this.helperSphereL1);
    this.helperSphereL1.parent = this.groupElbowLeft;
    this.helperSphereL1.position.x = 0.0;
    this.helperSphereL1.position.y = 0.0;
    this.helperSphereL1.position.z = 0.0;

    this.helperSphereR1 = new THREE.Mesh(sphereGeometry, sphereMaterial1);
    this.app.scene.add(this.helperSphereR1);
    this.helperSphereR1.parent = this.groupElbowRight;
    this.helperSphereR1.position.x = 0.0;
    this.helperSphereR1.position.y = 0.0;
    this.helperSphereR1.position.z = 0.0;


    var sphereMaterial2 = new THREE.MeshPhongMaterial({ specular: 0x333333, shininess: 5, color: 0x00ff00 });
    this.helperSphereL2 = new THREE.Mesh(sphereGeometry, sphereMaterial2);
    this.app.scene.add(this.helperSphereL2);

    this.helperSphereR2 = new THREE.Mesh(sphereGeometry, sphereMaterial2);
    this.app.scene.add(this.helperSphereR2);
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.setStaticElbow = function (static)
{
    this.staticElbow = static;
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.setShowHelpers = function (show)
{
    if (this.helperSphereL1)
        this.helperSphereL1.visible = show;

    if (this.helperSphereR1)
        this.helperSphereR1.visible = show;

    if (this.helperSphereL2)
        this.helperSphereL2.visible = show;

    if (this.helperSphereR2)
        this.helperSphereR2.visible = show;

    if (!show && this.meshHelperLeft) {
        this.app.scene.remove(this.meshHelperLeft);
        disposeObject(this.meshHelperLeft, false);
        this.meshHelperLeft = undefined;
    }

    if (!show && this.meshHelperRight) {
        this.app.scene.remove(this.meshHelperRight);
        disposeObject(this.meshHelperRight, false);
        this.meshHelperRight = undefined;
    }

    for (var i = 0; i < this.materials.length; i++) {
        if (this.materials && this.materials[i]) {
            this.materials[i].transparent = show;
            this.materials[i].needsUpdate = true;
        }
    }

    this.setObjectMaterial('Shoulder_Cover_Right_A', this.materials[show ? 3 : 0]);
    this.setObjectMaterial('Shoulder_Cover_Right_B', this.materials[show ? 4 : 0]);

    this.setObjectMaterial('Elbow_Cover_Right_A', this.materials[show ? 3 : 0]);
    this.setObjectMaterial('Elbow_Cover_Right_B', this.materials[show ? 4 : 0]);

    this.setObjectMaterial('Shoulder_Cover_Left_A', this.materials[show ? 3 : 0]);
    this.setObjectMaterial('Shoulder_Cover_Left_B', this.materials[show ? 4 : 0]);

    this.setObjectMaterial('Elbow_Cover_Left_A', this.materials[show ? 3 : 0]);
    this.setObjectMaterial('Elbow_Cover_Left_B', this.materials[show ? 4 : 0]);
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.setObjectMaterial = function(name, material)
{
    var obj = this.objects[name];
    if (isObject(obj))
        obj.material = material;
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.changeOpacity = function (opacity)
{
    for (var i = 0; i < this.materials.length; i++) {
        if (this.materials && this.materials[i]) {
            this.materials[i].opacity = opacity;
            this.materials[i].needsUpdate = true;
        }
    }
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.getWorldPosition = function (name)
{
    var obj = this.objects[name];

    if (obj == undefined)
        return null;

    var mat = obj.matrixWorld;
    var pos = new THREE.Vector3();

    pos.setFromMatrixPosition(mat);

    return pos;
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.createHelperTubes = function (re, rw, le, lw)
{
    if (!this.doneInitializing)
        return;

    if (this.meshHelperLeft) {
        this.app.scene.remove(this.meshHelperLeft);
        disposeObject(this.meshHelperLeft, false);
        this.meshHelperLeft = undefined;
    }

    if (this.meshHelperRight) {
        this.app.scene.remove(this.meshHelperRight);
        disposeObject(this.meshHelperRight, false);
        this.meshHelperRight = undefined;
    }

    var vec_le = le["vector"].clone().normalize().multiplyScalar(this.seg1_left_len);
    var vec_lw = lw["vector"].clone().normalize().multiplyScalar(this.seg2_left_len);
    var vec_re = re["vector"].clone().normalize().multiplyScalar(this.seg1_right_len);
    var vec_rw = rw["vector"].clone().normalize().multiplyScalar(this.seg2_right_len);

    var positions;
    var pos;

    //
    // left arm
    positions = [];
    pos = this.getWorldPosition("Upper_Arm_Left");
    positions.push(pos);
    pos = pos.clone().add(vec_le);
    positions.push(pos);
    pos = pos.clone().add(vec_lw);
    positions.push(pos);

    this.meshHelperLeft = this.createHelperTube(positions);
    this.app.scene.add(this.meshHelperLeft);

    //
    // right arm
    positions = [];
    pos = this.getWorldPosition("Upper_Arm_Right");
    positions.push(pos);
    pos = pos.clone().add(vec_re);
    positions.push(pos);
    pos = pos.clone().add(vec_rw);
    positions.push(pos);

    this.meshHelperRight = this.createHelperTube(positions);
    this.app.scene.add(this.meshHelperRight);
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.createHelperTube = function (positions)
{
    var curve = new THREE.CurvePath();

    for (var i = 0; i < (positions.length-1); i++) {
        var lineCurve = new THREE.LineCurve3(positions[i], positions[i + 1]);

        curve.add(lineCurve);
    }

    var extrudePath = curve;
    var extrusionSegments = 100;
    var radius = 6;
    var radiusSegments = 14;
    var closed = false;

    var tubeGeometry = new THREE.TubeBufferGeometry(extrudePath, extrusionSegments, radius, radiusSegments, closed);

    curve = undefined;

    var mesh = new THREE.Mesh(tubeGeometry, this.helperMaterial);

    return mesh;
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.to_sphere = function (v)
{
    var radius = v.length();
    var theta, phi;

    if (radius == 0)
        return [0.0, 0.0, 0.0]; 

    theta = Math.acos(v.z / radius);

    // phi is from - 180~+180
    if (v.x != 0.0)
        phi = Math.atan(v.y / v.x);
    else {
        if (v.y > 0)
            phi = THREE.MathUtils.degToRad(90);
        else
            phi = THREE.MathUtils.degToRad(-90);
    }

    if ((v.x < 0) && (v.y > 0))
        phi = THREE.MathUtils.degToRad(180) + phi;
    else if ((v.x < 0) && (v.y < 0))
        phi = THREE.MathUtils.degToRad(-180) + phi;
    else
        phi = phi;

    return [radius, theta, phi];
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.getFromShoulderSphericalCoords = function (radius, phi, theta)
{
    //
    // zyx shoulder coordinate space
    var z = radius * Math.cos(theta);
    var y = radius * Math.sin(theta) * Math.sin(phi);
    var x = radius * Math.sin(theta) * Math.cos(phi);

    return new THREE.Vector3(x, y, z);
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.getFromElbowSphericalCoords = function (radius, phi, theta)
{
    //
    // yxz elbow coordinate space
    var y = radius * Math.cos(theta);
    var x = radius * Math.sin(theta) * Math.sin(phi);
    var z = radius * Math.sin(theta) * Math.cos(phi);

    return new THREE.Vector3(x, y, z);
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.getFromCartesianCoords = function (v)
{
    var radius = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    var theta, phi;

    if (radius === 0) {
        theta = 0;
        phi = 0;
    } else {
        theta = Math.atan2(v.x, v.z);
        phi = Math.acos(THREE.MathUtils.clamp(v.y / radius, - 1, 1));
    }

    return [radius, theta, phi];
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.updateBody = function (re, rw, le, lw)
{
    if (!this.doneInitializing)
        return;

    var obj, theta, phi;

    if (this.app.params.showHelpers) {
        this.createHelperTubes(re, rw, le, lw);
    }

    {
        //
        // convert theta and phi world to local
        var v = this.getFromShoulderSphericalCoords(this.seg1_left_len, le["phi"], le["theta"]);
        var k = this.getFromCartesianCoords(v);

        theta = Math.PI - k[1];
        phi = k[2];

        obj = this.objects['Shoulder_Left'];
        if (isObject(obj)) {
            obj.rotation.order = 'XYZ';
            obj.rotation.z = theta;
        }

        obj = this.objects['Upper_Arm_Left'];
        if (isObject(obj)) {
           obj.rotation.order = 'XYZ';
            obj.rotation.y = phi;
        }
    }

    {
        //
        // convert theta and phi world to local
        this.groupElbowLeft.updateWorldMatrix(true, false);
        this.groupElbowLeft.updateMatrixWorld(true);
        var vElbowWorld = new THREE.Vector3().setFromMatrixPosition(this.groupElbowLeft.matrixWorld);

        var v = this.getFromElbowSphericalCoords(this.seg2_left_len, lw["phi"], lw["theta"]);
        var worldPos = vElbowWorld.clone().add(v);
        var vLocal = this.groupElbowLeft.worldToLocal(worldPos.clone());

        if ((this.app.params.showHelpers) && (this.helperSphereL2)) {
            this.helperSphereL2.parent = this.groupElbowLeft;
            this.helperSphereL2.position.x = vLocal.x;
            this.helperSphereL2.position.y = vLocal.y;
            this.helperSphereL2.position.z = vLocal.z;
        }

        var v = new THREE.Vector3(vLocal.x, vLocal.z, vLocal.y);    // re-map to xzy coordinate space
        var k = this.getFromCartesianCoords(v);

        theta = Math.PI / 2 + k[1];
        phi = - k[2];

        obj = this.objects['Elbow_Left'];
        if (isObject(obj)) {
            obj.rotation.order = 'XYZ';
            obj.rotation.y = (this.staticElbow) ? this.staticElbowAngleLeft : theta;
        }

        obj = this.objects['Hand_Left'];
        if (isObject(obj)) {
            obj.rotation.order = 'ZXY';
            obj.rotation.z = phi;
        }
    }

    {
        //
        // convert theta and phi world to local
        var v = this.getFromShoulderSphericalCoords(this.seg1_right_len, re["phi"], re["theta"]);
        var k = this.getFromCartesianCoords(v);

        theta = Math.PI + k[1];
        phi = Math.PI - k[2];

        var quaternion = new THREE.Quaternion();
        quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 0), v);

        obj = this.objects['Shoulder_Right'];
        if (isObject(obj)) {
            obj.rotation.order = 'XYZ';
            obj.rotation.z = theta;
        }

        obj = this.objects['Upper_Arm_Right'];
        if (isObject(obj)) {
            obj.rotation.order = 'XYZ';
            obj.rotation.y = phi;
        }
    }

    {
        //
        // convert theta and phi world to local
        this.groupElbowRight.updateWorldMatrix(true, false);
        this.groupElbowRight.updateMatrixWorld(true);
        var vElbowWorld = new THREE.Vector3().setFromMatrixPosition(this.groupElbowRight.matrixWorld);

        var v = this.getFromElbowSphericalCoords(this.seg2_right_len, rw["phi"], rw["theta"]);
        var worldPos = vElbowWorld.clone().add(v);
        var vLocal = this.groupElbowRight.worldToLocal(worldPos.clone());

        if ((this.app.params.showHelpers) && (this.helperSphereR2)) {
            this.helperSphereR2.parent = this.groupElbowRight;
            this.helperSphereR2.position.x = vLocal.x;
            this.helperSphereR2.position.y = vLocal.y;
            this.helperSphereR2.position.z = vLocal.z;
        }

        var v = new THREE.Vector3(vLocal.x, vLocal.z, vLocal.y);
        var k = this.getFromCartesianCoords(v);

        theta = Math.PI / 2 + k[1];
        phi = - k[2];

        obj = this.objects['Elbow_Right'];
        if (isObject(obj)) {
            obj.rotation.order = 'XYZ';
            obj.rotation.y = (this.staticElbow) ? this.staticElbowAngleRight : theta;
        }

        obj = this.objects['Hand_Right'];
        if (isObject(obj)) {
            obj.rotation.order = 'ZXY';
            obj.rotation.z = phi;
        }
    }
}
//--------------------------------------------------------------------------------------------
MSRAbot.prototype.update = function (clock)
{
}

