<template>
  <div>
    <v-row dense>
      <v-col>
        <RoutingMap @addPoint="addPoint" @clear="clear" class="elevation-4" />
      </v-col>
    </v-row>
  </div>
</template>

<script lang="ts">
import Vue from 'vue';
import { Component, Provide, ProvideReactive } from 'vue-property-decorator';

import RoutingMap from '@/components/Map.vue';
import { ICoordinate, IGridNode, Path } from '@/types';
import apiService from '@/api-service';

@Component({
  name: 'HomeView',
  components: {
    RoutingMap,
  },
})
export default class Routing extends Vue {
  @ProvideReactive('start')
  private start: IGridNode | null = null;
  @ProvideReactive('destination')
  private destination: IGridNode | null = null;
  @ProvideReactive('path')
  private path: Path | null = null;

  private clear() {
    this.start = null;
    this.destination = null;
    this.path = null;
  }

  private async addPoint(c: ICoordinate) {
    const snappedC = await apiService.snapNode(c);
    if (!this.start) {
      this.start = snappedC;
    } else if (!this.destination) {
      this.destination = snappedC;
      await this.fetchShortestPath();
    }
  }

  private async fetchShortestPath() {
    if (!this.start || !this.destination) {
      return;
    }
    this.path = await apiService.shortestPath(
      this.start.id,
      this.destination.id
    );
    // this.path = {
    //   coordinates: [this.start, this.destination],
    //   distance: 0,
    // };
  }
}
</script>
