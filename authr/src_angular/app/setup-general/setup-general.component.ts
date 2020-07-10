import { Component, OnInit, OnDestroy } from '@angular/core';
import { AuthrService } from "../services/authr.service";
import { MatSnackBar } from '@angular/material';
import { NestedTreeControl } from '@angular/cdk/tree';
import { MatTreeNestedDataSource } from '@angular/material/tree';

interface DataNode {
  name: string;
  children?: DataNode[];
}

@Component({
  selector: 'setup-general',
  templateUrl: './setup-general.component.html',
  styleUrls: ['./setup-general.component.scss']
})
export class SetupGeneralComponent {

  treeControl = new NestedTreeControl<DataNode>(node => node.children);
  treeSource = new MatTreeNestedDataSource<DataNode>();

  constructor(public authrService: AuthrService,
              public snackbar: MatSnackBar) {

      this.treeSource.data = authrService.plan?this.planToTree(authrService.plan):[{name:"No Data"}];
  }

  nodeToTree(key,data): DataNode {
    if (typeof data[key] === 'string' || typeof data[key] === 'number') {
      return {name:key,children:[{name:data[key]}]};
    } else {
      let node: DataNode = {name:key,children:[]}
      Object.keys(data[key]).forEach((childKey) => {
        try {
          node.children.push(this.nodeToTree(childKey,data[key]))
        } catch {}
      })
      return node;
    }
  }

  planToTree(data): DataNode[] {

    let level:DataNode[] = [];
    Object.keys(data).forEach((key) => {
      level.push(this.nodeToTree(key,data));
    })
    return level
  }

  hasChild = (_: number, node: DataNode) => !!node.children && node.children.length > 0;

}
