import { ModuleWithProviders, NgModule, Optional, SkipSelf } from '@angular/core';

import { CommonModule } from '@angular/common';

import { AuthrService, AuthrServiceConfig } from '../services/authr.service';


@NgModule({
  imports:      [ CommonModule ],
  declarations: [],
  exports:      [],
  providers:    [ AuthrService ]
})
export class CoreModule {
  constructor (@Optional() @SkipSelf() parentModule: CoreModule) {
    if (parentModule) {
      throw new Error(
        'CoreModule is already loaded. Import it in the AppModule only');
    }
  }

  static forRoot(config: AuthrServiceConfig): ModuleWithProviders {
    return {
      ngModule: CoreModule,
      providers: [
        {provide: AuthrServiceConfig}
      ]
    };
  }
}
