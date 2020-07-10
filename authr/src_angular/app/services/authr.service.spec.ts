import { TestBed } from '@angular/core/testing';

import { AuthrService } from './authr.service';

describe('AuthrService', () => {
  beforeEach(() => TestBed.configureTestingModule({}));

  it('should be created', () => {
    const service: AuthrService = TestBed.get(AuthrService);
    expect(service).toBeTruthy();
  });
});
