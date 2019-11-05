import 'ol/ol.css';
import Map from 'ol/Map';
import View from 'ol/View';
import Draw from 'ol/interaction/Draw';
import { fromLonLat, toLonLat } from 'ol/proj'
import { Tile as TileLayer, Vector as VectorLayer } from 'ol/layer';
import { OSM, Vector as VectorSource } from 'ol/source';
import { Circle as CircleStyle, Fill, Stroke, Style } from 'ol/style';
import Feature from 'ol/Feature';
import { Point } from 'ol/geom';
import LineString from 'ol/geom/LineString'
import Overlay from 'ol/Overlay';
import { toStringHDMS } from 'ol/coordinate';
import Select from 'ol/interaction/Select';
import { click } from 'ol/events/condition';
import { points, routes } from './ny-a-84786-6463'

var container = document.getElementById('popup');
var content = document.getElementById('popup-content');
var closer = document.getElementById('popup-closer');

var overlay = new Overlay({
  element: container,
});

closer.onclick = function() {
  overlay.setPosition(undefined);
  closer.blur();
  return false;
};

var raster = new TileLayer({
  source: new OSM()
});

let markers = [];

var source = new VectorSource();

var vector = new VectorLayer({
  source,
  style: new Style({
    image: new CircleStyle({
      radius: 1,
      stroke: new Stroke({color: 'yellow', width: 1})
    }),
    stroke: new Stroke({
      color: 'red',
      width: 3,
    })
  })
});

var routeSource = new VectorSource();

var routeVector = new VectorLayer({
  source: routeSource,
  style: new Style({
    stroke: new Stroke({
      color: 'red',
      width: 3,
    }),
    zIndex: 100
  })
});

var map = new Map({
  layers: [raster, vector, routeVector],
  target: 'map',
  overlays: [overlay],
  view: new View({
    center: fromLonLat([-73.966393, 40.786272]),
    zoom: 9
  })
});

// map.on('singleclick', function(evt) {
//   var coordinate = evt.coordinate;
//   var hdms = toStringHDMS(toLonLat(coordinate));

//   content.innerHTML = '<p>You clicked here:</p><code>' + hdms +
//       '</code>';
//   overlay.setPosition(coordinate);
// });

// map.addInteraction(new Draw({
//   source,
//   type: 'Point'
// }));
const draw = () => {
  const map = window.co;
  for (let i = 0; i < points.length; i++) {
    const id = points[i];
    const p = map[id];
    const x = p[0];
    const y = p[1];
    const f = new Feature({
      color: 'orange',
      geometry: new Point(fromLonLat([ x, y ]))
    });
    markers.push(f);
  }
  console.log(points.length, routes.length);
  const r = [];
  for (let i = 0; i < routes.length; i++) {
    const id = routes[i];
    const p = map[id];
    const x = p[0];
    const y = p[1];
    r.push(fromLonLat([ x, y ]))
  }
  routeSource.addFeature(new Feature({
    geometry: new LineString(r)
  }));
  source.addFeatures(markers);
}

document.getElementById('cofile').addEventListener('input', e => {
  const { files } = e.target;
  if (!files || !files.length) return;
  const file = files[0];
  const fileReader = new FileReader();
  fileReader.onload = f => {
    const arr = fileReader.result.split('\n');
    const co = {};
    for (let i = 0; i < arr.length; i++) {
      const s = arr[i].split(' ');
      if (s.length !== 4) continue;
      if (s[0] === 'c') continue;
      if (s[0] === 'v') {
        let id = s[1], x = s[2], y = s[3];
        co[id - 1] = [x / 1000000, y / 1000000];
        // const f = new Feature({
        //   geometry: new Point(fromLonLat([ x / 1000000, y / 1000000 ]))
        // });
        // f.setId(id - 1);
        // markers.push(f);
      }
    }
    window.co = co;
    // source.addFeatures(markers);
    draw();
  };
  fileReader.readAsText(file);
});

// var selectSingleClick = new Select();
var selectClick = new Select({
  condition: click
});
selectClick.on('select', function(e) {
  const features = e.target;
  const arr = features.getFeatures().getArray();
  console.log(arr.map(e => e.getId()))
});
map.addInteraction(selectClick);

window.source = source;
window.map = map;