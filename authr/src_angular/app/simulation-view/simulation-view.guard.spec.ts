import { TestBed, async, inject } from '@angular/core/testing';

import { SimulationViewGuard } from './simulation-view.guard';

describe('SimulationViewGuard', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [SimulationViewGuard]
    });
  });

  it('should ...', inject([SimulationViewGuard], (guard: SimulationViewGuard) => {
    expect(guard).toBeTruthy();
  }));
});
