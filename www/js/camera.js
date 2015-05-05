var camImageNr = 0; // Serial number of current image
var camFinished = new Array(); // References to img objects which have finished downloading
var camPaused = false;

function camCreateImageLayer() {
	var img = new Image();
	img.style.position = "absolute";
	img.style.zIndex = -1;
	img.onload = camImageOnload;
	//img.style.width = "100%";
	//img.style.height = "100%";
	img.src = "http://" + document.domain + ":5000/?action=snapshot&n=" + (++camImageNr);
	var webcam = document.getElementById("camera-secondary");
	webcam.insertBefore(img, webcam.firstChild);
}

// Two layers are always present (except at the very beginning), to avoid flicker
function camImageOnload() {
	this.style.zIndex = camImageNr; // Image camFinished, bring to front!
	while (1 < camFinished.length) {
		var del = camFinished.shift(); // Delete old image(s) from document
		del.parentNode.removeChild(del);
	}
	camFinished.push(this);
	if (!camPaused) camCreateImageLayer();
}

$(function() {
    camCreateImageLayer();
});