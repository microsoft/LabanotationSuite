//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------

'use strict';

var _app = null;

//--------------------------------------------------------------------------------------------
var APP = function (domTarget)
{
    this.self = this;
    this.container = domTarget;
    this.clock = null;
    this.renderer = null;
    this.scene = null;
    this.camera = null;
    this.enableStats = false;
    this.stats = null;

    this.nativeWidth = 0;
    this.nativeHeight = 0;

    this.screenWidth = window.innerWidth;
    this.screenHeight = window.innerHeight;

    this.hiding = null;

    this.eventDoneInit = null;

    //
    // objects & modules
    this.msrabot = null;
    this.labanotation = null;

    //
    // input events
    this.mouse = new THREE.Vector3(0, 0, 0);
    this.touches = [];
}
//--------------------------------------------------------------------------------------------

APP.prototype.constructor = APP;

//--------------------------------------------------------------------------------------------
APP.prototype.initialize = function (filename)
{
    this.setupRenderer();
    this.setupDomEvents();
    this.setupMiscellaneous();
    this.createDebugLabels();
    this.initializeScene();

    //
    // load default labanotation file
    this.loadNewFile(filename);
}
//--------------------------------------------------------------------------------------------
APP.prototype.setupRenderer = function ()
{
    this.clock = new THREE.Clock();

    this.renderer = new THREE.WebGLRenderer({ antialias: true });

    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.screenWidth, this.screenHeight);
    this.renderer.setClearColor(0x000080, 1);

    this.renderer.shadowMap.enabled = true;

    this.container.appendChild(this.renderer.domElement);

    this.nativeWidth = this.renderer.domElement.clientWidth;
    this.nativeHeight = this.renderer.domElement.clientHeight;

    //
    // stats
    if (this.enableStats) {
        this.stats = new Stats();
        this.container.appendChild(this.stats.dom);
    }
}
//--------------------------------------------------------------------------------------------
APP.prototype.setupDomEvents = function ()
{
    window.addEventListener('resize', this.onWindowResize.bind(this), false);

    document.addEventListener('keydown', this.onDocumentKeyDown.bind(this), false);

    document.addEventListener('mousedown', this.onDocumentMouseDown.bind(this), false);
    document.addEventListener('mousemove', this.onDocumentMouseMove.bind(this), false);
    document.addEventListener('mouseup', this.onDocumentMouseUp.bind(this), false);

    document.addEventListener('wheel', this.onDocumentMouseWheel.bind(this), false);

    document.addEventListener('touchstart', this.onDocumentTouchStart.bind(this), false);
    document.addEventListener('touchmove', this.onDocumentTouchMove.bind(this), false);
    document.addEventListener('touchend', this.onDocumentTouchEnd.bind(this), false);

    document.addEventListener('contextmenu', this.onContextMenu.bind(this), false);
}
//--------------------------------------------------------------------------------------------
APP.prototype.setupMiscellaneous = function ()
{
    var scope = this;

    //
    // dat.GUI
    var gui = new dat.GUI({ width: 400 });

    this.params = {
        animationSpeed: 1,
        opacity: 0.4,
        staticElbow: true,
        showHelpers: false,
    };


    var folder = gui.addFolder('MSRAbot');
    folder.add(this.params, 'staticElbow').onChange(function () {
        if (scope.msrabot)
            scope.msrabot.setStaticElbow(scope.params.staticElbow);
    });

    folder.add(this.params, 'showHelpers').onChange(function () {
        if (scope.msrabot)
            scope.msrabot.setShowHelpers(scope.params.showHelpers);
    });

    folder.add(this.params, 'opacity', 0.0, 1.0).step(0.01).onChange(function () {
        scope.changeMSRAbotOpacity(scope.params.opacity);
    });

    // folder.open();
}
//--------------------------------------------------------------------------------------------
APP.prototype.initializeScene = function ()
{
    var scope = this;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    this.camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 1, 10000);
    this.camera.position.set(0, 260, 600);
    this.camera.lookAt(0, 160, 0);
    this.scene.add(this.camera);

    this.scene.add(new THREE.AmbientLight(0x707070));

    //
    // lights
    {
        var intensity = 1.5;
        var distance = 0;
        var angle = Math.PI / 4;
        var penumbra = 0.5;
        var shadow_mapsize = 4096;
        var shadow_bias = -0.0000222;

        var light = new THREE.SpotLight(0x606060, intensity, distance, angle, penumbra);
        light.position.set(-430, 825, 1000);
        light.castShadow = true;
        light.shadow = new THREE.LightShadow(new THREE.PerspectiveCamera(120, 1, 20, 5000));
        light.shadow.bias = shadow_bias;
        light.shadow.mapSize.width = shadow_mapsize;
        light.shadow.mapSize.height = shadow_mapsize;

        this.scene.add(light);

        var light = new THREE.SpotLight(0x303030, intensity, distance, angle, penumbra);
        light.position.set(-430, 825, -1000);
        light.castShadow = true;
        light.shadow = new THREE.LightShadow(new THREE.PerspectiveCamera(120, 1, 20, 5000));
        light.shadow.bias = shadow_bias;
        light.shadow.mapSize.width = shadow_mapsize;
        light.shadow.mapSize.height = shadow_mapsize;
        this.scene.add(light);
    }

    //
    // create plane MSRAbot stands on
    var planeGeometry = new THREE.PlaneBufferGeometry(2000, 2000);
    var planeMaterial = new THREE.MeshPhongMaterial({ color: 0xf0f0f0, flatShading: true, side: THREE.FrontSide });
    var plane = new THREE.Mesh(planeGeometry, planeMaterial);
    planeGeometry.rotateX(-Math.PI / 2);
    plane.receiveShadow = true;
    this.scene.add(plane);

    var helper = new THREE.GridHelper(2000, 100);
    helper.position.y = plane.position.y + 1;
    helper.material.opacity = 0.25;
    helper.material.transparent = true;
    this.scene.add(helper);

    //
    // create camera and scene panning controls
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.damping = 0.2;
    this.controls.addEventListener('change', function () {
        scope.render();
    });
    this.controls.addEventListener('start', function () {
        scope.cancelHideTransorm();
    });
    this.controls.addEventListener('end', function () {
        scope.delayHideTransform();
    });

    this.controls.target.set(0, 160, 0);
    this.controls.update();

    this.transformControl = new THREE.TransformControls(this.camera, this.renderer.domElement);

    this.transformControl.showZ = false;
    this.transformControl.addEventListener('change', function () {
        scope.render();
    });
    this.transformControl.addEventListener('dragging-changed', function (event) {
        scope.controls.enabled = !event.value;
    });
    this.scene.add(this.transformControl);

    // Hiding transform situation is a little in a mess :()
    this.transformControl.addEventListener('change', function () {
        scope.cancelHideTransorm();
    });
    this.transformControl.addEventListener('mouseDown', function () {
        scope.cancelHideTransorm();
    });
    this.transformControl.addEventListener('mouseUp', function () {
        scope.delayHideTransform();
    });

    //
    // set up custom event used by labanotation and msrabot modules 
    // signaling they''re done initializing
    this.eventDoneInit = new CustomEvent('doneInit', { "bubbles": true, "cancelable": false });

    document.addEventListener('doneInit', function (e) {
        if ((scope.msrabot != null) && (scope.msrabot.doneInitializing) && (scope.labanotation != null) && (scope.labanotation.doneInitializing))
            scope.doneInitializing();
    });

    //
    // create MSRAbot module
    this.msrabot = new MSRAbot(this);
    if (this.msrabot != null)
        this.msrabot.initialize(plane.position);

    //
    // create labanotation handling module
    this.labanotation = new Labanotation(this);
    if (this.labanotation != null)
        this.labanotation.initialize();
}
//--------------------------------------------------------------------------------------------
APP.prototype.createDebugLabels = function ()
{
    var x = 10;
    var y = 16;

    this.dbgLabel1 = this.createDebugLabel('dbgLabel1', x, y);
    y += 28;
    //this.dbgLabel2 = this.createDebugLabel('dbgLabel2', x, y);
    //y += 28;
    //this.dbgLabel3 = this.createDebugLabel('dbgLabel3', x, y);
}
//--------------------------------------------------------------------------------------------
APP.prototype.createDebugLabel = function (name, x, y)
{
    var dbgLabel = document.createElement('div');

    dbgLabel.id = name;
    dbgLabel.textContent = '';
    dbgLabel.style.left = '' + x + 'px';
    dbgLabel.style.top = '' + y + 'px';
    dbgLabel.style.opacity = "0.8";
    dbgLabel.style.padding = "4px";
    dbgLabel.style.borderBottom = "1px solid #363636";
    dbgLabel.style.fontFamily = "Monospace";
    dbgLabel.style.color = "#000";
    dbgLabel.style.zIndex = 10002;
    dbgLabel.style.position = "fixed";

    document.body.appendChild(dbgLabel);

    return dbgLabel;
}
//--------------------------------------------------------------------------------------------
APP.prototype.doneInitializing = function ()
{
}
//--------------------------------------------------------------------------------------------
APP.prototype.loadNewFile = function (filename)
{
    if (this.labanotation != null)
        this.labanotation.loadNewFile(filename);
}
//--------------------------------------------------------------------------------------------
APP.prototype.changeMSRAbotOpacity = function (opacity)
{
    if (this.msrabot != null)
        this.msrabot.changeOpacity(opacity);
}
//--------------------------------------------------------------------------------------------
APP.prototype.importFile = function (files)
{
    if (files.length < 1)
        return;

    var scope = this;
    var map = {};
    var file = files[0];

    map[file.name] = file;

    var manager = new THREE.LoadingManager();
    manager.setURLModifier(function (url) {
        var file = filesMap[url];

        if (file) {
            console.log('Loading', url);
            return URL.createObjectURL(file);
        }

        return url;
     });

    for (var i = 0; i < files.length; i++) {
        scope.loadFile(files[i], manager);
    }
}
//--------------------------------------------------------------------------------------------
APP.prototype.loadFile = function (file, manager)
{
    var scope = this;
    var filename = file.name;
    var extension = filename.split('.').pop().toLowerCase();

    var reader = new FileReader();
    reader.addEventListener('progress', function (event) {
        var size = '(' + Math.floor(event.total / 1000).toFixed(0) + ' KB)';
        var progress = Math.floor((event.loaded / event.total) * 100) + '%';

        console.log('Loading', filename, size, progress);
    });

    if (extension != 'json') {
        alert("please import a file with extension .json");
        return;
    }

    //
    // read json file
    reader.addEventListener('load', function (event) {
        var contents = event.target.result;

        if (contents.indexOf('postMessage') !== - 1) {
            var blob = new Blob([contents], { type: 'text/javascript' });
            var url = URL.createObjectURL(blob);

            var worker = new Worker(url);

            worker.onmessage = function (event) {
                event.data.metadata = { version: 2 };
                scope.labanotation.loadLabanotationData(filename, event.data);
            };

            worker.postMessage(Date.now());

            return;
        }

        // >= 3.0
        var data;

        try {
            data = JSON.parse(contents);
        } catch (error) {
            alert(error);
            return;
        }

        scope.labanotation.loadLabanotationData(filename, data);

    }, false);

    reader.readAsText(file);
}
//--------------------------------------------------------------------------------------------
APP.prototype.delayHideTransform = function ()
{
    this.cancelHideTransorm();
    this.hideTransform();
}
//--------------------------------------------------------------------------------------------
APP.prototype.hideTransform = function ()
{
    var scope = this;

    this.hiding = setTimeout(function () {
        scope.transformControl.detach(scope.transformControl.object);
    }, 2500);
}
//--------------------------------------------------------------------------------------------
APP.prototype.cancelHideTransorm = function ()
{
    if (this.hiding) {
        clearTimeout(this.hiding);
        this.hiding = null;
    }
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentKeyDown = function (event)
{
    switch (event.keyCode) {
        case 0x41: // 'A'
            break;
        case 0x44: // 'D'
            break;
    }
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentMouseDown = function (event)
{
    // event.preventDefault();
    this.mouse.x = (event.clientX / this.nativeWidth) * 2 - 1;
    this.mouse.y = -(event.clientY / this.nativeHeight) * 2 + 1;
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentMouseMove = function (event)
{
    // event.preventDefault();
    this.mouse.x = (event.clientX / this.nativeWidth) * 2 - 1;
    this.mouse.y = -(event.clientY / this.nativeHeight) * 2 + 1;
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentMouseUp = function (event)
{
    // event.preventDefault();
    this.mouse.x = (event.clientX / this.nativeWidth) * 2 - 1;
    this.mouse.y = -(event.clientY / this.nativeHeight) * 2 + 1;
}
//--------------------------------------------------------------------------------------------
APP.prototype.convertTouchEvents = function (touches)
{
    this.touches = [];

    for (var i = 0; i < event.touches.length; i++) {
        this.touches.push(
            new THREE.Vector2(
                    ( touches[i].pageX / this.nativeWidth)  * 2 - 1,
                    (-touches[i].pageY / this.nativeHeight) * 2 + 1));
    }
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentTouchStart = function (event)
{
    // event.preventDefault();
    this.convertTouchEvents(event.touches);
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentTouchMove = function (event)
{
    // event.preventDefault();
    this.convertTouchEvents(event.touches);
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentTouchEnd = function (event)
{
    // event.preventDefault();
    this.convertTouchEvents(event.touches);
}
//--------------------------------------------------------------------------------------------
APP.prototype.onDocumentMouseWheel = function (event)
{
    // event.preventDefault();
}
//--------------------------------------------------------------------------------------------
APP.prototype.onContextMenu = function (event)
{
    // event.preventDefault();
}
//--------------------------------------------------------------------------------------------
APP.prototype.onWindowResize = function ()
{
    this.nativeWidth = this.renderer.domElement.clientWidth;
    this.nativeHeight = this.renderer.domElement.clientHeight;

    this.screenWidth = window.innerWidth;
    this.screenHeight = window.innerHeight;

    if (this.camera) {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
    }

    if (this.renderer)
        this.renderer.setSize(this.screenWidth, this.screenHeight);

    this.render();
}
//--------------------------------------------------------------------------------------------
APP.prototype.update = function ()
{
    var delta = this.clock.getDelta();

    TWEEN.update();

    if (this.stats)
        this.stats.update();

    if (this.labanotation != null)
        this.labanotation.update(this.clock);

    if (this.msrabot != null)
        this.msrabot.update(this.clock);
}
//--------------------------------------------------------------------------------------------
APP.prototype.render = function ()
{
    this.renderer.clear();

    this.renderer.render(this.scene, this.camera);
}
//--------------------------------------------------------------------------------------------
