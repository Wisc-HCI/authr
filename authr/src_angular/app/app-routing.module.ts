import { NgModule }             from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { SetupViewComponent }         from './setup-view/setup-view.component';
import { LoginViewComponent }         from './login-view/login-view.component';
import { PlanViewComponent }          from './plan-view/plan-view.component';
import { SimulationViewComponent }    from './simulation-view/simulation-view.component';
import { TaskDetailComponent }        from './task-detail/task-detail.component';
import { TherbligDetailComponent }    from './therblig-detail/therblig-detail.component';
import { MacroDetailComponent }       from './macro-detail/macro-detail.component';
import { TherbligPrimitiveDetailComponent } from './therblig-primitive-detail/therblig-primitive-detail.component';
import { SetupGeneralComponent }      from './setup-general/setup-general.component';
import { SetupAgentsComponent }       from './setup-agents/setup-agents.component';
import { SetupThingsComponent }       from './setup-things/setup-things.component';
import { SetupDestinationsComponent } from './setup-destinations/setup-destinations.component';

import { PlanViewGuard } from './plan-view/plan-view.guard';
import { SetupViewGuard } from './setup-view/setup-view.guard';
import { SimulationViewGuard } from './simulation-view/simulation-view.guard';

const routes: Routes = [
    { path: 'setup', component: SetupViewComponent, canActivate: [SetupViewGuard], children: [
        { path: 'general',component: SetupGeneralComponent, outlet: 'setup'},
        { path: 'agents',component: SetupAgentsComponent, outlet: 'setup'},
        { path: 'things',component: SetupThingsComponent, outlet: 'setup'},
        { path: 'destinations',component: SetupDestinationsComponent, outlet: 'setup'},
        { path: '', redirectTo: '/setup/(setup:general)', pathMatch: 'full'}
    ] },
    { path: 'plan', component: PlanViewComponent, canActivate: [PlanViewGuard], children: [
        { path: 'tasks/:task_id', component: TaskDetailComponent, outlet: 'config'},
        { path: 'therbligs/:therblig_id', component: TherbligDetailComponent, outlet: 'config'},
        { path: 'macros/:macro_id', component: MacroDetailComponent, outlet: 'config'},
        { path: 'therblig-primitives/:therblig_primitive_type', component: TherbligPrimitiveDetailComponent, outlet: 'config'}
    ]},
    { path: 'simulate', component: SimulationViewComponent, canActivate: [SimulationViewGuard]},
    { path: 'login', component: LoginViewComponent },
    { path: '', redirectTo: '/login', pathMatch: 'full'}
];

@NgModule({
    // Disable Tracing for final version
    imports: [ RouterModule.forRoot(routes, {enableTracing: false})],
    exports: [ RouterModule],
})

export class AppRoutingModule {}
