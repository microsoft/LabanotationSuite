//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
Labanotation = function (app)
{
    this.app = app;
    this.filename = null;
    this.labanotationName = "";
    this.duration = 0;
    this.animationStart = 0;
    this.start_time = 0;
    this.end_time = 0;
    this.animationTestIndex = 0;
    this.pausing = false;
    this.doneInitializing = false;
}

Labanotation.prototype.constructor = Labanotation;

//--------------------------------------------------------------------------------------------
Labanotation.prototype.initialize = function initLabanotation()
{
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.loadNewFile = function (filename)
{
    //var gesture_file = "/static/laban/" + filename + ".json";

    this.loadLabanotation(filename);
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.loadLabanotation = function (filename)
{
    var scope = this;
    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function (event) {
        if (xhr.readyState == XMLHttpRequest.DONE) {
            if (xhr.status == 200 || xhr.status == 0) {
                if (xhr.responseText.length > 0) {
                    var data = undefined;

                    try {
                        data = JSON.parse(xhr.responseText);
                    } catch (error) {
                        console.warn("FAILED to parse Labanotation buffer.");
                        console.error(error);
                    }

                    if (data) {
                        scope.loadLabanotationData(filename, data);

                        scope.doneInitializing = true;

                        //
                        // done. Fire init complete event
                        if (scope.app.eventDoneInit != null)
                            document.dispatchEvent(scope.app.eventDoneInit);
                    }

                    data = undefined;
                }
            } else {
                console.error("Couldn't load Labanotation json file [code " + xhr.status + "]");
            }
        }
    }

    xhr.addEventListener("abort", function () {
        console.warn("XMLHttpRequest abort.");
    }, false);

    xhr.addEventListener('error', function (event) {
        console.warn("XMLHttpRequest error.");
    }, false);

    xhr.overrideMimeType("text/xml;");
    xhr.open("GET", filename, true);

    xhr.send(null);
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.loadLabanotationData = function (filename, data)
{
    this.parseLabanotation(data);
    this.filename = filename;
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.parseLabanotation = function (data)
{
    var scope = this;

    this.labanotation = null;
    this.labanotationName = "";
    this.animationStart = 0;
    this.duration = 0;

    //
    // get node with labanotation information
    for (var key in data) {
        if (data.hasOwnProperty(key)) {
            scope.labanotation = data[key]
            scope.labanotationName = key;

            var min, max;

            for (var position in scope.labanotation) {
                if (scope.labanotation.hasOwnProperty(position)) {
                    var obj = scope.labanotation[position];
                    var time = obj["start time"];

                    if (min === undefined) {
                        min = time;
                        max = time;
                    } else {
                        min = Math.min(min, time);
                        max = Math.max(max, time);
                    }

                    scope.convertLabanotation2Vec(obj, "right elbow");
                    scope.convertLabanotation2Vec(obj, "right wrist");
                    scope.convertLabanotation2Vec(obj, "left elbow");
                    scope.convertLabanotation2Vec(obj, "left wrist");
                }
            }

            scope.start_time = min;
            scope.end_time = max;
            scope.duration = (max - min);
        }
    }
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.convertLabanotation2Vec = function (data, key)
{
    var obj = data[key];
    var dir = obj[0].toLowerCase();
    var lv = obj[1].toLowerCase();
    var phi = 0;
    var theta = 180;

    switch (lv) {
        case "high":
            theta = 45;
            break;
        case "normal":
            theta = 90;
            break;
        case "low":
            theta = 135;
            break;
        default:
            theta = 180
            console.log("Unknown level '" + lv + "' for '" + key + "'.");
            break;
    }

    switch (dir) {
        case "forward":
            phi = 0;
            break;
        case "right forward":
            phi = -45;
            break;
        case "right":
            phi = -90;
            break;
        case "right backward":
            phi = -135;
            break;
        case "backward":
            phi = 180;
            break;
        case "left backward":
            phi = 135;
            break;
        case "left":
            phi = 90;
            break;
        case "left forward":
            phi = 45;
            break;
        case "place":
            if (lv == "high") {
                theta = 5;
                phi = 0;
            } else if (lv == "low") {
                theta = 175;
                phi = 0;
            } else {
                theta = 180;
                phi = 0;
                console.log("Unknown place for '" + key + "'.");
            }
            break;
        default:
            phi = 0;
            console.log("Unknown direction for '" + key + "'.");
            break;
    }

    theta = THREE.MathUtils.degToRad(theta);
    phi = THREE.MathUtils.degToRad(phi);

    // length of the limb
    y = Math.cos(theta);
    x = Math.sin(theta) * Math.sin(phi);
    z = Math.sin(theta) * Math.cos(phi);

    //
    // write data back into object
    obj["theta"] = theta;
    obj["phi"] = phi;
    obj["vector"] = new THREE.Vector3(x, y, z);
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.getRotation = function (theta, phi)
{
    var result = [];

    result["theta"] = theta;
    result["phi"] = phi;

    var y = Math.cos(theta);
    var x = Math.sin(theta) * Math.sin(phi);
    var z = Math.sin(theta) * Math.cos(phi);

    result["vector"] = new THREE.Vector3(x, y, z);

    return result;
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.interpolateRotation = function (obj1, obj2, s, name)
{
    var theta, phi;

    if (obj1 == null) {
        theta = obj2[name]["theta"];
        phi = obj2[name]["phi"];
    } else {
        theta = THREE.MathUtils.lerp(obj1[name]["theta"], obj2[name]["theta"], s);
        phi = THREE.MathUtils.lerp(obj1[name]["phi"], obj2[name]["phi"], s);
    }

    return this.getRotation(theta, phi);
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.updateAnimation = function (time)
{
    var scope = this;
    var objPrevious = null;
    var timePrevious = 0;

    for (var position in scope.labanotation) {
        if (scope.labanotation.hasOwnProperty(position)) {
            var obj = scope.labanotation[position];

            var start_time = obj["start time"];

            if ((timePrevious <= time) && (time <= start_time)) {
                var s = (time - timePrevious) / (start_time - timePrevious)

                var re = this.interpolateRotation(objPrevious, obj, s, "right elbow");
                var rw = this.interpolateRotation(objPrevious, obj, s, "right wrist");
                var le = this.interpolateRotation(objPrevious, obj, s, "left elbow");
                var lw = this.interpolateRotation(objPrevious, obj, s, "left wrist");

                if (scope.app.msrabot) {
                    scope.app.msrabot.updateBody(re, rw, le, lw);
                }

                return;
            }

            objPrevious = obj;
            timePrevious = start_time;
        }
    }
}
//--------------------------------------------------------------------------------------------
Labanotation.prototype.update = function (clock)
{
    if (!this.doneInitializing)
        return;

    var elapsedTime = clock.getElapsedTime() * 1000.0;
    if (this.pausing){
        this.animationStart = elapsedTime;
        if (this.app.dbgLabel1)
        var timeIndex = (elapsedTime - this.animationStart) * this.app.params.animationSpeed;
        this.app.dbgLabel1.textContent = "time: " + timeIndex.toFixed(0) + "ms, duration: " + this.duration.toFixed(0) + "ms";
    } else {
        if (this.animationStart == 0)
            this.animationStart = elapsedTime;

        var timeIndex = (elapsedTime - this.animationStart) * this.app.params.animationSpeed;

        if (timeIndex > this.duration) {
            // pause
            this.pausing = true;
            // reset
            //timeIndex = 0
            //this.animationStart = elapsedTime;
        }

        if (this.app.dbgLabel1)
            this.app.dbgLabel1.textContent = "time: " + timeIndex.toFixed(0) + "ms, duration: " + this.duration.toFixed(0) + "ms";

        this.updateAnimation(this.start_time + timeIndex);
    }
}


Labanotation.prototype.pause = function ()
{
    this.pausing  = true;
}
Labanotation.prototype.resume = function ()
{
    this.pausing  = false;
}