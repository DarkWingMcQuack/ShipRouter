<template>
  <div class="map-container">
    <l-map
      :zoom="zoom"
      :center="center"
      @click="handleLeftClick"
      @update:zoom="updateZoom"
      @update:center="updateCenter"
      style="z-index: 1"
      ref="map"
    >
      <l-control-layers position="topright"></l-control-layers>
      <l-tile-layer
        v-for="provider in tileProviders"
        :key="provider.name"
        :name="provider.name"
        :visible="provider.visible"
        :attribution="provider.attribution"
        :url="provider.url"
        layer-type="base"
      />
      <l-control position="bottomleft">
        <v-card elevation="4">
          <v-btn @click="clearMap" title="Remove waypoints" icon text>
            <v-icon>mdi-delete</v-icon>
          </v-btn>
        </v-card>
      </l-control>
      <l-polyline v-if="path" :lat-lngs="path.coordinates">
        <l-tooltip>
          <strong>Distance: {{ (path.distance / 1000).toFixed(2) }}km</strong>
        </l-tooltip>
      </l-polyline>
      <l-marker v-if="start" :lat-lng="start"></l-marker>
      <l-marker v-if="destination" :lat-lng="destination"></l-marker>
    </l-map>
  </div>
</template>

<script lang="ts">
import Vue from 'vue';
import Component from 'vue-class-component';
import { Inject, InjectReactive, Watch } from 'vue-property-decorator';

import L, { LatLng, LeafletMouseEvent } from 'leaflet';
import {
  LMap,
  LControl,
  LControlLayers,
  LTileLayer,
  LPolyline,
  LMarker,
  LTooltip,
} from 'vue2-leaflet';

import { ICoordinate, IGridNode, Path } from '@/types';

@Component({
  name: 'MapComponent',
  components: {
    LMap,
    LControl,
    LControlLayers,
    LTileLayer,
    LPolyline,
    LMarker,
    LTooltip,
  },
})
export default class RoutingMap extends Vue {
  private tileProviders = [
    {
      name: 'OpenStreetMap',
      visible: true,
      url: 'http://{s}.tile.osm.org/{z}/{x}/{y}.png',
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    },
    {
      name: 'Thunderforest Landscape',
      visible: false,
      url:
        'https://{s}.tile.thunderforest.com/landscape/{z}/{x}/{y}.png?apikey=bd08b205580548d18e6235e2da754318',
      attribution:
        '&copy; <a href="http://www.thunderforest.com/">Thunderforest</a>, &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    },
  ];
  private zoom: number = 1;
  private center: ICoordinate = { lat: 48.74703, lng: 9.1046 };

  @InjectReactive('start')
  private start!: IGridNode | null;
  @InjectReactive('destination')
  private destination!: IGridNode | null;
  @InjectReactive('path')
  private path!: Path | null;

  private updateZoom(zoomValue: number) {
    this.zoom = zoomValue;
  }
  private updateCenter(centerValue: ICoordinate) {
    this.center = centerValue;
  }
  private handleLeftClick(e: LeafletMouseEvent) {
    this.$emit('addPoint', e.latlng);
  }
  private clearMap() {
    this.$emit('clear');
  }
}
</script>

<style lang="scss" scoped>
.map-container {
  height: 700px;
}
</style>
