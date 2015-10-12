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

L.NumberedDetectionIcon = L.Icon.extend({
	options: {
    // EDIT THIS TO POINT TO THE FILE AT http://www.charliecroom.com/marker_hole.png (or your own marker)
    iconUrl: 'css/markers/marker_poi.png',
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
    rctMarkers : [(List of markers for lawnmower)],
    splMarkers : [(List of markers for spiral)],
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
       * @param [in] readonly Whether or not this marker is editable.
       * @param [in] icf The icon factory to use, if any.
       */
      this.addNumberedMarker = function(markerList, location, draggable, readonly, icf) {
        var iconFactory = icf ? icf :
          draggable ? L.NumberedDivIconRed : L.NumberedDivIcon;
        var icon = new iconFactory({number : (markerList.length + 1)});
        var marker = new L.marker(location, {
          draggable: draggable,
          icon: icon
        }).addTo(data.map);

        //Set altitude default
        marker.alt = 0;
        
        markerList.push(marker);
        if (!readonly) {
          marker.on('click', function(event) {
            if (data.editMode) {
              data.map.removeLayer(marker);
              markerList.splice($.inArray(marker, markerList), 1);
              instance.toggleEditMarkers(markerList, true);
              $(instance).trigger("wptUpdated");
            }
            instance.updateBounds();
            instance.updateWptPath();
            instance.updateExclusions();
          });
        }

        marker.on('drag', function(e) {
          instance.updateBounds();
          instance.updateWptPath();
          instance.updateExclusions();
          $(instance).trigger("wptUpdated");
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
       * Adds a detection marker to the image
       * @param [in] detection The detection data.
       */
      this.addDetection = function(detection) {
        var icon = {};
        var latlng = L.latLng(detection.lat, detection.lon);
        instance.addNumberedMarker(data.dctMarkers, latlng, false, true, L.NumberedDetectionIcon);
        var marker = data.dctMarkers[data.dctMarkers.length-1];
        marker.bindPopup("<img src=\""+detection.image+"\" style='width:100%;'>" + 
          "<br>Time: " + detection.timestamp +
          "<br>At: " + detection.lat + ", " + detection.lon +
          "<br>Detection altitude: " +
          detection.alt + "m");
      }
      
      /**
       * Add a detection track to the map.
       * @param [in] track The track positon.
       */
      this.addDetectionTrack = function(track) {
        if (!data.paths.copterTrack) {
          instance.addPath('copterTrack', 'red');
        }
        
        var latlng = L.latLng(track.lat, track.lon);
        data.paths.copterTrack.addLatLng(latlng);
      }
      
      /**
       * Add a new exclusion zone.
       */
      this.addExclusionZone = function() {
        data.excIndex++;
        data.excMarkers.length = data.excIndex;
        data.excMarkers[data.excIndex] = [];
      }
      
      /**
       * Retrieve the current user's position.
       * @param [in] ret The object to store the return value in.
       */
      this.getUserPosition = function(ret) {
        if (data.markers.userMarker) {
          var latlng = data.markers.userMarker.getLatLng();
          ret.lat = latlng.lat;
          ret.lon = latlng.lng;
        }
      }

      /**
       * Update the user's position on the map.
       * @param [in] latitude The user's latitude.
       * @param [in] longitude The user's longitude.
       */
      this.updateUserPosition = function(latitude, longitude) {
        var latlng = L.latLng(latitude, longitude);
        if (!data.markers.userMarker) {
          instance.addMarker('userMarker', 'css/markers/marker-user.png', latlng, false);
        } else {
          data.markers.userMarker.setLatLng(latlng).update();
        }
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
       * Jumps to the specified location.
       * @param [in] latitude A latitude, "copter", or "user".
       * @param [in] longitude A longitude. Optional if "copter" or "user"
       *                       is specified.
       */
      this.jumpTo = function(latitude, longitude) {
        if (latitude === "copter") {
          data.map.panTo(data.markers.copterMarker.getLatLng());
        } else if (latitude === "user") {
          if (data.markers.userMarker) {
            data.map.panTo(data.markers.userMarker.getLatLng());
          }
        } else {
          data.map.panTo(new L.LatLng(latitude, longitude));
        }
      }

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
        $(instance).trigger("wptUpdated");
      };
      
      /**
       * Clear the detection markers.
       */
      this.clearDetectionMarkers = function() {
        instance.clearMarkers(data.dctMarkers);
        instance.removeMapLayer(data.map, data.paths, 'copterTrack');
      }

      /**
       * Removes the currently displayed markers.
       */
      this.clearActiveMarkers = function() {
        if (data.pattern === "manual") {
          instance.clearMarkers(data.wptMarkers);
          instance.updateWptPath();
        } else if (data.pattern === "lawnmower") {
          instance.clearMarkers(data.rctMarkers);
          instance.updateBounds();
        } else if (data.pattern === "spiral") {
          instance.clearMarkers(data.splMarkers);
          instance.updateBounds();
        } else if (data.pattern === "exclusion") {
          for (var i = 0; i < data.excMarkers.length; i++) {
            instance.clearMarkers(data.excMarkers[i]);
          }
          data.excMarkers.length = 1;
          data.excMarkers[0].length = 0;
          data.excIndex = 0;
          instance.updateExclusions();
        }
      }

      /**
       * Get the active marker coordinates.
       * @param [in] callback The callback to pass the list of coordinates to.
       * @param [in] force Call the callback even if in edit mode.
       */
      this.getActiveMarkerCoordinates = function(callback, force) {
        var packageCoordinates = function (data) {
          var newdata = [];
          if (typeof data != 'undefined') {
            $.each(data, function(index, val) {
              var ll = val.getLatLng();
              newdata.push([ll.lat,ll.lng, val.alt]);
            });
          }
          return newdata;
        };

        if (!data.editMode || force) { //Don't send if we're editing...
          if (data.pattern === "manual") {
            callback(data.pattern, packageCoordinates(data.wptMarkers));
          } else if (data.pattern === "lawnmower") {
            callback(data.pattern, packageCoordinates(data.rctMarkers));
          } else if (data.pattern === "spiral") {
            callback(data.pattern, packageCoordinates(data.splMarkers));
          } else if (data.pattern === "exclusion") {
            callback(data.pattern, packageCoordinates(data.excMarkers[0]));
          }
        }
      };
      
      /**
       * Retrieve the list of exclusion zones.
       * @param [in] callback The callback receiving the exclusion zones.
       */
      this.getExclusionZones = function(callback) {
        var pkg = [];
        for (var i = 0; i < data.excMarkers.length; i++) {
          var zone = [];
          for (var j = 0; j < data.excMarkers[i].length; j++) {
            var ll = data.excMarkers[i][j].getLatLng();
            zone.push([ll.lat, ll.lng, 0]);
          }
          if (zone.length > 0) {
            pkg.push(zone);
          }
        }
        
        //Always call callback. No exclusion zones may be what's required.
        callback(pkg);
      }
      
      /**
       * Updates a waypoint position and information
       * @param [in] index The zero-index of the marker/waypoint.
       * @param [in] type The type of the value to update (lat/lon/alt)
       * @param [in] value The value to update with.
       */
      this.updateWaypoint = function(index, type, value) {
        if (data.editMode) {
          var markers;
          if (data.pattern === "manual") {
            markers = data.wptMarkers;
          } else if (data.pattern === "lawnmower") {
            markers = data.rctMarkers;
          } else if (data.pattern === "spiral") {
            markers = data.splMarkers;
          } else if (data.pattern === "exclusion") {
            markers = data.excMarkers[data.excIndex];
          }
          
          if (markers && index < markers.length) {
            var ll = markers[index].getLatLng();
            if (type === "lat") {
              ll.lat = value;
            } else if (type === "lon") {
              ll.lng = value;
            } else if (type === "alt") {
              markers[index].alt = value;
            }
            markers[index].setLatLng(ll).update();
            instance.updateWptPath();
            instance.updateBounds();
            instance.updateExclusions();
          }
        }
      }

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
          markerList[index].alt = value.alt;
        });
        $(instance).trigger("wptUpdated");
      };

      /**
       * Toggles the edit mode of the map.
       * @param [in] pattern The pattern which we are editing.
       */
      this.toggleEditMode = function(pattern) {
        if (pattern === "off") {
          pattern = data.pattern;
          data.editMode = false;
        } else {
          data.editMode = !data.editMode;
        }

        if (pattern === "manual") {
          instance.toggleEditMarkers(data.wptMarkers, data.editMode);
        } else if (pattern === "lawnmower") {
          instance.toggleEditMarkers(data.rctMarkers, data.editMode);
        } else if (pattern === "spiral") {
          instance.toggleEditMarkers(data.splMarkers, data.editMode);
        } else if (pattern === "exclusion") {
          var nm = [];
          //Make marker list contiguous (don't allow for empty marker lists)
          for (var i = 0; i < data.excMarkers.length; i++) {
            if (data.excMarkers[i].length > 0) {
              instance.toggleEditMarkers(data.excMarkers[i], data.editMode);
              nm.push(data.excMarkers[i]);
            }
          }
          if (nm.length === 0) {
            nm.length = 1;
            nm[0] = [];
          }
          data.excMarkers = nm;
          data.excIndex = nm.length-1;
        }
        data.pattern = pattern;
      }

      /**
       * Updates the bounds rectangle shown for the lawnmower pattern.
       * @param [in] data The data for the map instance.
       */
      this.updateBounds = function() {
        instance.removeMapLayer(data.map, data, 'boundRegion');
        instance.removeMapLayer(data.map, data.paths, 'lawnmowerOverlay');
        instance.removeMapLayer(data.map, data, 'upperBoundRegion');

        if (data.pattern === "lawnmower") {
          if (data.rctMarkers.length == 2) {
            function calcLawnmower(ll1, ll2) {
              //Good for small distances only.
              var SWEEP_SPACING = 3; //3m
              //Determine which way we are sweeping
              var d1 = ll1.distanceTo(L.latLng(ll1.lat, ll2.lng));
              var d2 = L.latLng(ll1.lat, ll2.lng).distanceTo(ll2);
              var points = Math.floor(Math.min(d1, d2)/SWEEP_SPACING);
              var pts = [];
              
              if (points != 0) {
                  var modlat = true;
                  var frac;
                  
                  if (d1 > d2) {
                    frac = (ll2.lat - ll1.lat) / points;
                  } else {
                    frac = (ll2.lng - ll1.lng) / points;
                    modlat = false;
                  }
                  for (var i = 0; i < points; i++) {
                    v1 = modlat ? L.latLng(ll1.lat + frac*i, ll1.lng) :
                                  L.latLng(ll1.lat, ll1.lng + frac*i);
                    v2 = modlat ? L.latLng(v1.lat, ll2.lng) : 
                                  L.latLng(ll2.lat, v1.lng);
                    if (i%2) {
                      pts.push(v2);
                      pts.push(v1);
                    } else {
                      pts.push(v1);
                      pts.push(v2);
                    }
                  }
                  if (!(points%2)) {
                    pts.push(modlat ? L.latLng(ll2.lat, ll1.lng) :
                                      L.latLng(ll1.lat, ll2.lng));
                  }
              } else {
                  pts.push(ll1);
              }
              pts.push(ll2);
              
              return pts;
            }
            
            data.boundRegion = L.rectangle(L.latLngBounds(
              data.rctMarkers[0].getLatLng(), data.rctMarkers[1].getLatLng()),
              {color : "#0066FF", weight : 1}
            ).addTo(data.map);
            instance.addPath('lawnmowerOverlay', 'green',
              calcLawnmower(data.rctMarkers[0].getLatLng(),
              data.rctMarkers[1].getLatLng()));
          }
        } else if (data.pattern === "spiral") {
          if (data.splMarkers.length >= 2) {
            data.boundRegion = L.circle(data.splMarkers[0].getLatLng(),
              data.splMarkers[0].getLatLng().distanceTo(data.splMarkers[1].getLatLng()),
              {color : "#0033FF", weight : 1}
            ).addTo(data.map);
          }
          if (data.splMarkers.length >= 3) {
            data.upperBoundRegion = L.circle(data.splMarkers[0].getLatLng(),
              data.splMarkers[0].getLatLng().distanceTo(data.splMarkers[2].getLatLng()),
              {color : "#FF3300", weight : 1}
            ).addTo(data.map);
          }
        }
      };
      
      /**
       * Updates the exclusion zone polygons that are displayed.
       */
      this.updateExclusions = function() {
        instance.removeMapLayer(data.map, data, 'exclusionZones');
        var lls = [];
        
        for (var i = 0; i < data.excMarkers.length; i++) {
          var ll = [];
          for (var j = 0; j < data.excMarkers[i].length; j++) {
            ll.push(data.excMarkers[i][j].getLatLng());
          }
          lls.push(ll);
        }
        
        if (typeof L.multiPolygon === "undefined") {
          data.exclusionZones = L.polygon(lls, {'color' : "#AA2233", 'fillColor' : "#FF0055"}).addTo(data.map);
        } else {
          data.exclusionZones = L.multiPolygon(lls, {'color' : "#AA2233", 'fillColor' : "#FF0055"}).addTo(data.map);
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
          hide(data.rctMarkers, data.map);
          hide(data.splMarkers, data.map);
          for (var i = 0; i < data.excMarkers.length; i++)
            hide(data.excMarkers[i], data.map);
          instance.removeMapLayer(data.map, data.paths, 'lawnmowerOverlay');
          instance.removeMapLayer(data.map, data.paths, 'wptPath');
          instance.removeMapLayer(data.map, data, 'boundRegion');
          instance.removeMapLayer(data.map, data, 'upperBoundRegion');
          //Always display exclusion zones since they affect everything
          //for (var i = 0; i < data.exclusionZones.length; i++)
          //  instance.removeMapLayer(data.map, data, 'exclusionZones');
        } else if (pattern === "manual") {
          hide(data.wptMarkers, data.map);
          instance.removeMapLayer(data.map, data.paths, 'wptPath');
        } else if (pattern === "lawnmower") {
          hide(data.rctMarkers, data.map);
          instance.removeMapLayer(data.map, data, 'boundRegion');
        } else if (pattern === "spiral") {
          hide(data.splMarkers, data.map);
          instance.removeMapLayer(data.map, data, 'boundRegion');
          instance.removeMapLayer(data.map, data, 'upperBoundRegion');
        } else if (pattern === "exclusion") {
          for (var i = 0; i < data.excMarkers.length; i++)
            hide(data.excMarkers[i], data.map);
          //instance.removeMapLayer(data.map, data, 'exclusionZones');
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
          instance.toggleEditMarkers(data.wptMarkers, data.editMode);
          instance.updateWptPath();
        } else if (pattern === "lawnmower") {
          show(data.rctMarkers, data.map);
          instance.toggleEditMarkers(data.rctMarkers, data.editMode);
          instance.updateBounds();
        } else if (pattern === "spiral") {
          show(data.splMarkers, data.map);
          instance.toggleEditMarkers(data.splMarkers, data.editMode);
          instance.updateBounds();
        } else if (pattern === "exclusion") {
          if (data.editMode) {
            for (var i = 0; i < data.excMarkers.length; i++) {
              show(data.excMarkers[i], data.map);
              instance.toggleEditMarkers(data.excMarkers[i], data.editMode);
            }
          }
          instance.updateExclusions();
        }
        $(instance).trigger("wptUpdated");
      };

      /**
       * Utility function to remove a given item from the map and update our structure as well.
       * @param [in] map The map.
       * @param [in] obj The object holding the item to be removed.
       * @param [in] item The name of the item to be removed.
       */
      this.removeMapLayer = function(map, obj, item) {
        if (obj[item]) {
          if (obj[item].constructor === Array) {
            for (var i = 0; i < obj[item].length; i++) {
              map.removeLayer(obj[item][i]);
            }
            obj[item].length = 0;
          } else {
            map.removeLayer(obj[item]);
            obj[item] = undefined;
          }
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
          rctMarkers : [],
          splMarkers : [],
          dctMarkers : [],
          excMarkers : [[]],
          paths : {},
          editMode : false,
          pattern : undefined,
          exclusionZones : [],
          excIndex : 0
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
              if (data.rctMarkers.length < 2) {
                instance.addNumberedMarker(data.rctMarkers, pos.latlng, true);
              }
              instance.updateBounds();
            } else if (data.pattern === "spiral") {
              if (data.splMarkers.length < 3) {
                instance.addNumberedMarker(data.splMarkers, pos.latlng, true);
              }
              instance.updateBounds();
            } else if (data.pattern === "exclusion") {
              instance.addNumberedMarker(data.excMarkers[data.excIndex], pos.latlng, true);
              instance.updateExclusions();
            }
            $(instance).trigger("wptUpdated");
          }
        });

        instance.addMarker('copterMarker', 'css/markers/helicopter.png', UWA, false);
        instance.addPath('copterPath', 'red');
        instance.addPath('wptPath', 'blue');
      }
    });
  };
}(jQuery));