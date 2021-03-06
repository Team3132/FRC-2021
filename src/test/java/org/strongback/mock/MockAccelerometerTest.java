/*
 * Strongback
 * Copyright 2015, Strongback and individual contributors by the @authors tag.
 * See the COPYRIGHT.txt in the distribution for a full listing of individual
 * contributors.
 *
 * Licensed under the MIT License; you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://opensource.org/licenses/MIT
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.strongback.mock;



import org.junit.jupiter.api.BeforeEach;

/**
 * @author Randall Hauch
 */
public class MockAccelerometerTest extends AbstractDoubleMockTest {

    private MockAccelerometer mock;

    @BeforeEach
    public void beforeEach() {
        mock = new MockAccelerometer();
        initialize(mock::setAcceleration, mock::getAcceleration);
    }
}
