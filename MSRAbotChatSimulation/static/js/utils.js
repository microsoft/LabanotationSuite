//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
(function ()
{
    if (typeof window.CustomEvent === "function") return false;

    function CustomEvent(event, params) {
        params = params || { bubbles: false, cancelable: false, detail: undefined };
        var evt = document.createEvent('CustomEvent');
        evt.initCustomEvent(event, params.bubbles, params.cancelable, params.detail);
        return evt;
    }

    CustomEvent.prototype = window.Event.prototype;

    window.CustomEvent = CustomEvent;
})();
//--------------------------------------------------------------------------------------------
function isObject(val)
{
    if (val === null) { return false; }
    return ((typeof val === 'function') || (typeof val === 'object'));
}
//--------------------------------------------------------------------------------------------
function disposeMaterial(mat)
{
    if (!mat)
        return;

    if (mat.map) {
        mat.map.dispose();
        mat.map = undefined;
    }

    if (mat.materials) {
        for (i = 0; i < mat.materials.length; i++) {
            if (mat.materials[i].map) {
                mat.materials[i].map.dispose();
                mat.materials[i].map = undefined;
            }

            mat.materials[i].dispose();
        }
    } else {
        mat.dispose();
    }

    mat = undefined;
}
//--------------------------------------------------------------------------------------------
function disposeObject(obj, disposeMaterial)
{
    if (obj !== null) {
        for (var i = 0; i < obj.children.length; i++) {
            disposeObject(obj.children[i]);
        }

        if (obj.geometry) {
            obj.geometry.dispose();
            obj.geometry = undefined;
        }

        if (disposeMaterial && obj.material) {
            disposeMaterial(obj.material, disposeMaterial);
            obj.material = undefined;
        }

        if (obj.texture) {
            obj.texture.dispose();
            obj.texture = undefined;
        }
    }
    obj = undefined;
}
//--------------------------------------------------------------------------------------------
