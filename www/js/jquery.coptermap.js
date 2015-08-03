/**
 * @file jquery.coptermap.js
 * @brief Code to display the map using Leaflet. Written as a jQuery plugin.
 * @author Jeremy Tan, 20933708 (Adapted from Alexander Mazur; 2014 group)
 */

/*******************************LEAFLET MARKERS********************************/

/**
 * A numbered marker (pin shaped).
 */
L.NumberedDivIcon = L.Icon.extend({
	options: {
    // EDIT THIS TO POINT TO THE FILE AT http://www.charliecroom.com/marker_hole.png (or your own marker)
    iconUrl: 'css/markers/marker_hole.png',
    number: '',
    shadowUrl: null,
    iconSize: new L.Point(25, 41),
		iconAnchor: new L.Point(13, 41),
		popupAnchor: new L.Point(0, -33),
		/*
		iconAnchor: (Point)
		popupAnchor: (Point)
		*/
		className: 'leaflet-div-icon'
	},

	createIcon: function () {
		var div = document.createElement('div');
		var img = this._createImg(this.options['iconUrl']);
		var numdiv = document.createElement('div');
		numdiv.setAttribute ( "class", "number" );
		numdiv.innerHTML = this.options['number'] || '';
		div.appendChild ( img );
		div.appendChild ( numdiv );
		this._setIconStyles(div, 'icon');
		return div;
	},

	//you could change this to add a shadow like in the normal marker if you really wanted
	createShadow: function () {
		return null;
	}
});

/**
 * A numbered marker (pin shaped). Displays a red '-'; to distinguish edit mode.
 */
L.NumberedDivIconRed = L.Icon.extend({
	options: {
    // EDIT THIS TO POINT TO THE FILE AT http://www.charliecroom.com/marker_hole.png (or your own marker)
    iconUrl: 'css/markers/marker_hole_red.png',
    number: '',
    shadowUrl: null,
    iconSize: new L.Point(25, 41),
		iconAnchor: new L.Point(13, 41),
		popupAnchor: new L.Point(0, -33),
		/*
		iconAnchor: (Point)
		popupAnchor: (Point)
		*/
		className: 'leaflet-div-icon'
	},

	createIcon: function () {
		var div = document.createElement('div');
		var img = this._createImg(this.options['iconUrl']);
		var numdiv = document.createElement('div');
		numdiv.setAttribute ( "class", "number" );
		numdiv.innerHTML = this.options['number'] || '';
		div.appendChild ( img );
		div.appendChild ( numdiv );
		this._setIconStyles(div, 'icon');
		return div;
	},

	//you could change this to add a shadow like in the normal marker if you really wanted
	createShadow: function () {
		return null;
	}
});

/*********************************PLUGIN START*********************************/

/* Data structure
  var mapData = {
    map : (The Leaflet map object returned by L.map),
    editMode : (Boolean indicating if we're in edit mode),
    markers : {(Dictionary of markers by name (persistent))},
    wptMarkers : [(List of markers for waypoints)],
    bndMarkers : [(List of markers for bounds)],
    paths : {(Dictionary of paths by name)}
  }
*/

(function($) {
  $.fn.copterMap = function(options) {
    /** Lat/Lng of UWA (around James Oval) **/
    var UWA = [-31.979839, 115.817546];
    //Options to pass to function calls. Apparently slicing arguments is bad.
    var args = [];
    for (var i = 1; i < arguments.length; i++) {
        args[i-1] = arguments[i];
    }

    return this.each(function () {
      var instance = this;
      var data = $(instance).data("copterMap");
      //Initialisation code at the very bottom of this function.

      /**
       * Add a marker to the map.
       * @param [in] name The name of the marker.
       * @param [in] iconUrl The url to the image to be used for the marker.
       * @param [in] location Lat/Lng of marker (as array)
       * @param [in] draggable Boolean indicating if this marker is draggable.
       */
      this.addMarker = function(name, iconUrl, location, draggable) {
        var icon = L.icon({
          iconUrl : iconUrl,
          iconSize : [32, 37],
          iconAnchor : [16, 35]
        });

        data.markers[name] = L.marker(location, {
          draggable : draggable,
          icon : icon
        }).addTo(data.map);
      };

      /**
       * Add a numbered marker to the map (for waypoints/setting bounds).
       * @param [in] markerList The list of markers to add this new one to.
       * @param [in] location The location to add the marker at.
       * @param [in] draggable Whether or not this marker is draggable.
       */
      this.addNumberedMarker = function(markerList, location, draggable) {
        var iconFactory = draggable ? L.NumberedDivIconRed : L.NumberedDivIcon;
        var icon = new iconFactory({number : (markerList.length + 1)});
        var marker = new L.marker(location, {
          draggable: draggable,
          icon: icon
        }).addTo(data.map);

        markerList.push(marker);
        marker.on('click', function(event) {
          if (data.editMode) {
            data.map.removeLayer(marker);
            markerList.splice($.inArray(marker, markerList), 1);
            instance.toggleEditMarkers(markerList, true);
          }
          instance.updateBounds();
          instance.updateWptPath();
        });

        marker.on('drag', function(e) {
          instance.updateBounds();
          instance.updateWptPath();
        });
      };

      /**
       * Adds a new path (polyline) to the map.
       * @param [in] name The name of the path.
       * @param [in] colour The colour of the path.
       * @param [in,opt] coords A list of coords for the path.
       */
      this.addPath = function(name, colour, coords) {
        if (coords) {
          data.paths[name] = L.polyline(coords, {color: colour}).addTo(data.map);
        } else {
          data.paths[name] = L.polyline([], {color: colour}).addTo(data.map);
        }
      };

      /**
       * Retrieve the current user's position.
       * @param [in] ret The object to store the return value in.
       */
      this.getUserPosition = function(ret) {
        if (data.markers.userMarker) {
          var latlng = data.markers.userMarker.getLatLng();
          ret.lat = latlng.lat;
          ret.lon = latlng.lon;
        }
      }

      /**
       * Update the user's position on the map.
       * @param [in] latitude The user's latitude.
       * @param [in] longitude The user's longitude.
       */
      this.updateUserPosition = function(latitude, longitude) {
        var latlng = L.latLng(latitude, longitude);
        data.markers.userMarker.setLatLng(latlng).update();
      };

      /**
       * Update the copter's position on the map.
       * @param [in] latitude Copter latitude
       * @param [in] longitude Copter longitude
       */
      this.updateCopterPosition = function(latitude, longitude) {
        var latlng = L.latLng(latitude, longitude);
        data.markers.copterMarker.setLatLng(latlng).update();
        if (data.paths.copterPath) {
          data.paths.copterPath.addLatLng(latlng);
        }
      };

      /**
       * Clears the displayed path that the copter has travelled.
       */
      this.clearCopterPath = function () {
        instance.removeMapLayer(data.map, data.paths, 'copterPath');
        instance.addPath('copterPath', 'red');
      };

      /**
       * Toggles the display of the path that the copter has travelled.
       */
      this.toggleCopterPath = function () {
        if (data.paths.copterPath) {
          instance.removeMapLayer(data.map, data.paths, 'copterPath');
        } else {
          instance.addPath('copterPath', 'red');
        }
      };

      /**
       * Clear a marker list and remove those markers from the map.
       * @param [in] markerList The list of markers to remove.
       */
      this.clearMarkers = function(markerList) {
        for (var i = 0; i < markerList.length; i++) {
          data.map.removeLayer(markerList[i]);
        }
        markerList.length = 0;
      };

      /**
       * Removes the currently displayed markers.
       */
      this.clearActiveMarkers = function() {
        if (data.pattern === "manual") {
          instance.clearMarkers(data.wptMarkers);
          instance.updateWptPath();
        } else if (data.pattern == "lawnmower") {
          instance.clearMarkers(data.bndMarkers);
          instance.updateBounds();
        }
      }

      /**
       * Get the active marker coordinates.
       * @param [in] callback The callback to pass the list of coordinates to.
       */
      this.getActiveMarkerCoordinates = function(callback) {
        var packageCoordinates = function (data) {
          var newdata = [];
          if (typeof data != 'undefined') {
            $.each(data, function(index, val) {
              lat = val.getLatLng().lat;
              lng = val.getLatLng().lng;
              newdata.push([lat,lng]);
            });
          }
          return newdata;
        };

        if (!data.editMode) { //Don't send if we're editing...
          if (data.pattern === "manual") {
            callback(data.pattern, packageCoordinates(data.wptMarkers));
          } else if (data.pattern == "lawnmower") {
            callback(data.pattern, packageCoordinates(data.bndMarkers));
          }
        }
      };

      /**
       * Toggle the display of the draggable waypoint markers.
       * @param [in] markerList The list of markers to toggle.
       * @param [in] editMode Boolean indicating whether to enable edit mode or not.
       */
      this.toggleEditMarkers = function(markerList, editMode) {
        var tmpmarkers = markerList.slice();
        instance.clearMarkers(markerList);
        $.each(tmpmarkers, function(index, value){
          instance.addNumberedMarker(markerList, value.getLatLng(), editMode);
        });
      };

      /**
       * Toggles the edit mode of the map.
       * @param [in] pattern The pattern which we are editing.
       */
      this.toggleEditMode = function(pattern) {
        if (pattern === "off" && data.pattern) {
          pattern = data.pattern;
          data.editMode = false;
        } else {
          data.editMode = !data.editMode;
        }

        if (pattern === "manual") {
          instance.toggleEditMarkers(data.wptMarkers, data.editMode);
        } else if (pattern === "lawnmower") {
          instance.toggleEditMarkers(data.bndMarkers, data.editMode);
        }
        data.pattern = pattern;
      }

      /**
       * Updates the bounds rectangle shown for the lawnmower pattern.
       * @param [in] data The data for the map instance.
       */
      this.updateBounds = function() {
        instance.removeMapLayer(data.map, data, 'boundRect');

        if (data.pattern == "lawnmower") {
          if (data.bndMarkers.length == 2) {
            data.boundRect = L.rectangle(L.latLngBounds(
              data.bndMarkers[0].getLatLng(), data.bndMarkers[1].getLatLng()),
              {color : "#0066FF", weight : 1}
            ).addTo(data.map);
          }
        }
      };

      /**
       * Updates the waypoint path displayed
       * @param [in] instance The map instance.
       * @param [in] data The data associated with this instance.
       */
      this.updateWptPath = function() {
        instance.removeMapLayer(data.map, data.paths, 'wptPath');

        if (data.pattern  === "manual") {
          var coords = [];
          $.each(data.wptMarkers, function(index, value) {
            coords.push(value.getLatLng());
          });
          instance.addPath('wptPath', 'blue', coords);
        }
      };

      /**
       * Hides the markers for the given type.
       * @param [in] pattern The type of markers to show (e.g. 'lawnmower' or 'manual')
       */
      this.hideMarkers = function(pattern) {
        var hide = function(markerList, map) {
          for (var i = 0; i < markerList.length; i++) {
            map.removeLayer(markerList[i]);
          }
        };

        if (pattern === "all") {
          hide(data.wptMarkers, data.map);
          hide(data.bndMarkers, data.map);
          instance.removeMapLayer(data.map, data.paths, 'wptPath');
          instance.removeMapLayer(data.map, data, 'boundRect');
        } else if (pattern === "manual") {
          hide(data.wptMarkers, data.map);
          instance.removeMapLayer(data.map, data.paths, 'wptPath');
        } else if (pattern === "lawnmower") {
          hide(data.bndMarkers, data.map);
          instance.removeMapLayer(data.map, data, 'boundRect');
        }
      };

      /**
       * Shows the markers for the given type.
       * @param [in] pattern The type of markers to show (e.g. 'lawnmower' or 'manual')
       */
      this.showMarkers = function(pattern) {
        var show = function(markerList, map) {
          for (var i = 0; i < markerList.length; i++) {
            markerList[i].addTo(map);
          }
        };

        data.pattern = pattern;
        if (pattern === "manual") {
          show(data.wptMarkers, data.map);
          instance.updateWptPath();
        } else if (pattern === "lawnmower") {
          show(data.bndMarkers, data.map);
          instance.updateBounds();
        }
      };

      /**
       * Utility function to remove a given item from the map and update our structure as well.
       * @param [in] map The map.
       * @param [in] obj The object holding the item to be removed.
       * @param [in] item The name of the item to be removed.
       */
      this.removeMapLayer = function(map, obj, item) {
        if (obj[item]) {
          map.removeLayer(obj[item]);
          obj[item] = undefined;
        }
      }

      //PLUGIN INITIALISATION AND HANDLING CODE
      if (typeof(this[options]) === 'function') {
        //We're calling a function.
        this[options].apply(this, args);
      } else {
        //We're initialising our plugin.
        var map = L.map($(instance)[0]).setView(UWA, 18);
        data = $.extend({
          map : map,
          markers : {},
          wptMarkers : [],
          bndMarkers : [],
          paths : {},
          editMode : false,
          pattern : undefined
        }, options);
        $(instance).data("copterMap", data);

        L.tileLayer('tiles/sat/gs_{x}_{y}_{z}.jpg', {
          maxZoom: 21,
          minZoom: 17
        }).addTo(map);

        map.on('click', function(pos) {
          if (data.editMode) {
            if (data.pattern === "manual") {
              instance.addNumberedMarker(data.wptMarkers, pos.latlng, true);
              instance.updateWptPath();
            } else if (data.pattern === "lawnmower") {
              if (data.bndMarkers.length < 2) {
                instance.addNumberedMarker(data.bndMarkers, pos.latlng, true);
                instance.updateBounds();
              } else {
                instance.updateBounds();
              }
            }
          }
        });

        instance.addMarker('copterMarker', 'css/markers/helicopter.png', UWA, false);
        instance.addPath('copterPath', 'red');
        instance.addPath('wptPath', 'blue');

        if (navigator.geolocation) {
          instance.addMarker('userMarker', 'css/markers/marker-user.png', UWA, false);
        }
      }
    });
  };
}(jQuery));